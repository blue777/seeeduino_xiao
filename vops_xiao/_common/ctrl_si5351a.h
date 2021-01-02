#ifndef __CTRL_SI5351A_H_INCLUDED__
#define __CTRL_SI5351A_H_INCLUDED__


#include <cstdint>
#include <algorithm>
#include "ctrl_i2c.h"


class ctrl_Si5351a_audio
{
protected:
	class PLL
	{
	public:
		PLL(uint32_t crystal_freq, uint32_t fvco_freq)
		{
			// Fvco freq = Crystal freq * (a + b/c)
			uint32_t	fvco_frac = fvco_freq % crystal_freq;
			uint32_t	fvco_gcd = ctrl_Si5351a_audio::GetGCD(crystal_freq, fvco_frac);
			uint16_t	fvco_a = fvco_freq / crystal_freq;
			uint32_t	fvco_b = fvco_frac / fvco_gcd;
			uint32_t	fvco_c = crystal_freq / fvco_gcd;

			while (0xFFFFF < fvco_c)
			{
				fvco_c >>= 1;
				fvco_b >>= 1;
			}

			// MSNx_P1[17:0] = 128 * a +     (128 * b / c) - 512
			// MSNx_P2[19:0] = 128 * b - c * (128 * b / c)
			// MSNx_P3[19:0] = c
			_P1 = 128 * fvco_a + (128 * fvco_b / fvco_c) - 512;
			_P2 = 128 * fvco_b - (128 * fvco_b / fvco_c) * fvco_c;
			_P3 = fvco_c;
		}

	public:
		uint32_t	_P1;
		uint32_t	_P2;
		uint32_t	_P3;
	};

	class CLK
	{
	public:
		CLK(uint32_t fvco_freq, uint32_t freq)
		{
			// Determine R-Div
			{
				uint32_t	div = fvco_freq / freq;

				_Rdiv = 0;
				while (0 == (1 & div))
				{
					_Rdiv++;
					div >>= 1;
				}

				while (7 < _Rdiv)
				{
					_Rdiv--;
					div <<= 1;
				}

				freq *= 1 << _Rdiv;
			}

			// MultiSynth freq = fvco_freq / ((a+b/c) * rdiv)
			uint32_t	clk_frac = fvco_freq % freq;
			uint32_t	clk_gcd = ctrl_Si5351a_audio::GetGCD(freq, clk_frac);
			uint32_t	clk_a = fvco_freq / freq;
			uint32_t	clk_b = clk_frac / clk_gcd;
			uint32_t	clk_c = freq / clk_gcd;

			while (0xFFFFF < clk_c)
			{
				clk_c >>= 1;
				clk_b >>= 1;
			}

			// MSx_P1[17:0] = 128 * a + floor(128*b/c)-512
			// MSx_P2[19:0] = 128 * b - c * floor(128*b/c)
			// MSx_P3[19:0] = c
			_P1 = 128 * clk_a + (128 * clk_b / clk_c) - 512;
			_P2 = 128 * clk_b - (128 * clk_b / clk_c) * clk_c;
			_P3 = clk_c;
		}

	public:
		uint8_t		_Rdiv;
		uint32_t	_P1;
		uint32_t	_P2;
		uint32_t	_P3;
	};

public:
	ctrl_Si5351a_audio( uint32_t nCrystalFreq=25000000, uint8_t nCrystalLoad = 8 ) :
		m_i2c( 0x60 ),
		m_nCrystalFreq(nCrystalFreq),
		m_nCrystalLoad(nCrystalLoad)
	{
	}

	void	begin(uint32_t sampling_freq, uint16_t clk0_fs, uint16_t clk1_fs, uint16_t clk2_fs)
	{
		uint16_t	max_fs = std::max({ clk0_fs, clk1_fs, clk2_fs });
		uint32_t	min_div = 900000000 / (sampling_freq * max_fs);
		uint32_t	fvco_freq = sampling_freq * max_fs * min_div;
		PLL			pll(m_nCrystalFreq, fvco_freq);

		// Disable all clock
		m_i2c.write({ 0x0003, 0xFF });
		m_i2c.write({ 0x0010, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 });

		m_i2c.write({ 0x0002, 0x53 });	// Interrupt Mask
		m_i2c.write({ 0x0007, 0x00 });
		m_i2c.write({ 0x000F, 0x00 });	// PLL config
			// 7:6	= CLKIN_DIV
			// 3    = PLLB_SRC
			// 2	= PLLA_SRC

		/* MSNA PLL section */
		m_i2c.write({ 0x001A,
			(0xFF & (pll._P3 >> 8)),	// 7:0	= MSNA_P3[15:8]
			(0xFF & (pll._P3 >> 0)),	// 7:0	= MSNA_P3[7:0]
			(0x03 & (pll._P1 >> 16)),	// 1:0	= MSNA_P1[17:16]
			(0xFF & (pll._P1 >> 8)),	// 7:0	= MSNA_P1[15:8]
			(0xFF & (pll._P1 >> 0)),	// 7:0	= MSNA_P1[7:0]	
			(0xF0 & (pll._P3 >> 12)) |	// 7:4	= MSNA_P3[19:16]
			(0x0F & (pll._P2 >> 16)),	// 3:0	= MSNA_P2[19:16]
			(0xFF & (pll._P2 >> 8)),	// 7:0	= MSNA_P2[15:8]
			(0xFF & (pll._P2 >> 0)) });	// 7:0	= MSNA_P2[7:0]

		/* MS0 section */
		if (0 < clk0_fs)
		{
			CLK		clk0(fvco_freq, sampling_freq * clk0_fs);

			m_i2c.write({ 0x0010, 0x0F });	// CLK0 Control
				// 7	= Clock 0 Power Down. 
				// 6	= MultiSync 0 Integer Mode
				// 5	= MultiSynth Source Select for CLK0.
				// 4	= Output Clock 0 Invert
				// 3:2	= Output Clock 0 Input Source. 
				// 1:0	= CLK0 Output Rise and Fall time / Drive Strength Control.

			m_i2c.write({ 0x002A,
				0xFF & (clk0._P3 >> 8),		// 7:0 = MSx_P3[15:8]
				0xFF & (clk0._P3 >> 0),		// 7:0 = MSx_P3[7:0]
				0x70 & (clk0._Rdiv << 4) |	// 6:4 = Rx_DIV[2:0]
				0 |							// 3:2 = MSx_DIVY4[1:0]
				0x03 & (clk0._P1 >> 16),	// 1:0 = MSx_P1[17:16]
				0xFF & (clk0._P1 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk0._P1 >> 0),		// 7:0 = MSx_P1[7:0]
				0xF0 & (clk0._P3 >> 12) |	// 7:4 = MSx_P3[19:16]
				0x0F & (clk0._P2 >> 16),	// 3:0 = MSx_P2[19:16]
				0xFF & (clk0._P2 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk0._P2 >> 0) });	// 7:0 = MSx_P1[7:0]
		}

		/* MS1 section */
		if (0 < clk1_fs)
		{
			CLK		clk1(fvco_freq, sampling_freq * clk1_fs);

			m_i2c.write({ 0x0011, 0x0F });	// CLK1 Control
				// 7	= Clock 0 Power Down. 
				// 6	= MultiSync 0 Integer Mode
				// 5	= MultiSynth Source Select for CLK0.
				// 4	= Output Clock 0 Invert
				// 3:2	= Output Clock 0 Input Source. 
				// 1:0	= CLK0 Output Rise and Fall time / Drive Strength Control.

			m_i2c.write({ 0x0032,
				0xFF & (clk1._P3 >> 8),		// 7:0 = MSx_P3[15:8]
				0xFF & (clk1._P3 >> 0),		// 7:0 = MSx_P3[7:0]
				0x70 & (clk1._Rdiv << 4) |	// 6:4 = Rx_DIV[2:0]
				0 |							// 3:2 = MSx_DIVY4[1:0]
				0x03 & (clk1._P1 >> 16),	// 1:0 = MSx_P1[17:16]
				0xFF & (clk1._P1 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk1._P1 >> 0),		// 7:0 = MSx_P1[7:0]
				0xF0 & (clk1._P3 >> 12) |	// 7:4 = MSx_P3[19:16]
				0x0F & (clk1._P2 >> 16),	// 3:0 = MSx_P2[19:16]
				0xFF & (clk1._P2 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk1._P2 >> 0) });	// 7:0 = MSx_P1[7:0]
		}

		/* MS2 section */
		if (0 < clk2_fs)
		{
			CLK		clk2(fvco_freq, sampling_freq * clk2_fs);

			m_i2c.write({ 0x0012, 0x0F });	// CLK2 Control
				// 7	= Clock 0 Power Down. 
				// 6	= MultiSync 0 Integer Mode
				// 5	= MultiSynth Source Select for CLK0.
				// 4	= Output Clock 0 Invert
				// 3:2	= Output Clock 0 Input Source. 
				// 1:0	= CLK0 Output Rise and Fall time / Drive Strength Control.

			m_i2c.write({ 0x003A,
				0xFF & (clk2._P3 >> 8),		// 7:0 = MSx_P3[15:8]
				0xFF & (clk2._P3 >> 0),		// 7:0 = MSx_P3[7:0]
				0x70 & (clk2._Rdiv << 4) |	// 6:4 = Rx_DIV[2:0]
				0 |							// 3:2 = MSx_DIVY4[1:0]
				0x03 & (clk2._P1 >> 16),	// 1:0 = MSx_P1[17:16]
				0xFF & (clk2._P1 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk2._P1 >> 0),		// 7:0 = MSx_P1[7:0]
				0xF0 & (clk2._P3 >> 12) |	// 7:4 = MSx_P3[19:16]
				0x0F & (clk2._P2 >> 16),	// 3:0 = MSx_P2[19:16]
				0xFF & (clk2._P2 >> 8),		// 7:0 = MSx_P1[15:8]
				0xFF & (clk2._P2 >> 0) });	// 7:0 = MSx_P1[7:0]
		}
			
		m_i2c.write({ 0x005A, 0x00, 0x00});	// MS6_P1, MS7_P1
		m_i2c.write({ 0x0095, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });	// 0x95 - 0x9B : Spread Spectrum
		m_i2c.write({ 0x00A2, 0x00, 0x00, 0x00 });	// 0xA2 - 0xA4 : VCXO
		m_i2c.write({ 0x00B7, (unsigned char)(0x12 |
			((m_nCrystalLoad <= 6) ?
				0x40 :
				(10 <= m_nCrystalLoad) ?
					0xC0 :
					0x80)) });

		// PLL soft reset
		m_i2c.write({ 0x00B1, 0xAC });

		// Enable all clock
		m_i2c.write({ 0x0003, 0xF8 });
		return;
	}

	void	end()
	{
		// Disable all clock
		m_i2c.write({ 0x0003, 0xFF });

		// Power down all clock
		m_i2c.write({ 0x0010,
			0x8C,	// CLK0 Control
			0x8C,	// CLK1 Control
			0x8C,	// CLK2 Control
			0x8C,	// CLK3 Control
			0x8C,	// CLK4 Control
			0x8C,	// CLK5 Control
			0x8C,	// CLK6 Control
			0x8C });// CLK7 Control
			// 7	= Clock 0 Power Down. 
			// 6	= MultiSync 0 Integer Mode
			// 5	= MultiSynth Source Select for CLK0.
			// 4	= Output Clock 0 Invert
			// 3:2	= Output Clock 0 Input Source. 
			// 1:0	= CLK0 Output Rise and Fall time / Drive Strength Control.
	}

protected:
	static	uint32_t	GetGCD(uint32_t a, uint32_t b)	// greatest common divisor
	{
		uint32_t	t;

		if (a < b)
		{
			t = a;
			a = b;
			b = t;
		}

		while (0 < b)
		{
			t = a;
			a = b;
			b = t % b;
		}

		return	a;
	}

protected:
	ctrl_i2c		m_i2c;
	const uint32_t	m_nCrystalFreq;
	const uint8_t	m_nCrystalLoad;
};

#endif // __CTRL_SI5351A_H_INCLUDED__