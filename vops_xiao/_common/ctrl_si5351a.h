#ifndef __CTRL_SI5351A_H_INCLUDED__
#define __CTRL_SI5351A_H_INCLUDED__


#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <bitset>
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
		CLK(uint32_t fvco_freq, uint32_t freq, int RDiv = 0)
		{
			_Rdiv = 0;

			// R-Div: 1,2,4・・・64,128
			// MSDiv: 8 ~ 2048
			// Determine R-Div
			if( (0 < RDiv) && (RDiv <= 128) )
			{
				while (1 < RDiv)
				{
					_Rdiv++;
					RDiv >>= 1;
				}
			}
			else
			{
				uint32_t	div = fvco_freq / freq;
				while ( (0 == (1 & div)) && (16 <= div) && (_Rdiv < 7) )
				{
					_Rdiv++;
					div >>= 1;
				}
			}

			freq *= 1 << _Rdiv;

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

		bool	IsMultiSynthCompati(CLK& b)
		{
			return false;
			return	(_P1 == b._P1) &&
					(_P2 == b._P2) &&
					(_P3 == b._P3);
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

	void	begin(uint32_t sampling_freq, int16_t clk0_fs, int16_t clk1_fs, int16_t clk2_fs)
	{
		uint8_t		rev = 0;
		m_i2c.write({ 0x00 });
		m_i2c.read(&rev,1);

		uint32_t	max_fvco = ((rev & 3) == 0) ? 720000000 : 900000000;
		uint16_t	lcm_fs01 = GetLCM(abs(clk0_fs), abs(clk1_fs));
		uint16_t	lcm_fs02 = GetLCM(abs(clk0_fs), abs(clk2_fs));
		uint16_t	lcm_fs   = GetLCM(lcm_fs01, lcm_fs02);
		uint32_t	min_div = max_fvco / (sampling_freq * lcm_fs);	// Make even to use integer mode
		uint32_t	fvco_freq = sampling_freq * lcm_fs * min_div;
		PLL			pll(m_nCrystalFreq, fvco_freq);

//		printf("fvco = %d, MCLK = %d\n", fvco_freq, lcm_fs * sampling_freq);

		int rdiv[3] = { 0 };
		if ((std::bitset<32>(lcm_fs).count() == 1) && (lcm_fs <= 128))
		{
			rdiv[0] = lcm_fs / abs(clk0_fs);
			rdiv[1] = lcm_fs / abs(clk1_fs);
			rdiv[2] = lcm_fs / abs(clk2_fs);
		}
		else if ((std::bitset<32>(lcm_fs01).count() == 1) && (lcm_fs01 <= 128))
		{
			rdiv[0] = lcm_fs01 / abs(clk0_fs);
			rdiv[1] = lcm_fs01 / abs(clk1_fs);
			rdiv[2] = 0;
		}
		else if ((std::bitset<32>(lcm_fs02).count() == 1) && (lcm_fs02 <= 128))
		{
			rdiv[0] = lcm_fs02 / abs(clk0_fs);
			rdiv[1] = 0;
			rdiv[2] = lcm_fs02 / abs(clk2_fs);
		}

		CLK		clk0(fvco_freq, sampling_freq * abs(clk0_fs), rdiv[0]);
		CLK		clk1(fvco_freq, sampling_freq * abs(clk1_fs), rdiv[1]);
		CLK		clk2(fvco_freq, sampling_freq * abs(clk2_fs), rdiv[2]);
		uint8_t	CLKxCTRL[3];

		CLKxCTRL[0] = 0x43 | (clk0_fs < 0 ? 0x10 : 0x00) | (clk0_fs == 0 ? 0x80 : 0x00) | 0x0C;	
		CLKxCTRL[1] = 0x43 | (clk1_fs < 0 ? 0x10 : 0x00) | (clk1_fs == 0 ? 0x80 : 0x00) | (clk1.IsMultiSynthCompati(clk0) ? 0x08 : 0x0C);
		CLKxCTRL[2] = 0x43 | (clk2_fs < 0 ? 0x10 : 0x00) | (clk2_fs == 0 ? 0x80 : 0x00) | (clk2.IsMultiSynthCompati(clk0) ? 0x08 : 0x0C);
			// 7	= Clock 0 Power Down. 
			// 6	= MultiSync 0 Integer Mode
			// 5	= MultiSynth Source(PLLA or PLLB) Select for CLKx.
			// 4	= Output Clock X Invert
			// 3:2	= Output Clock X Input Source. 
			//		0 : XTAL
			//		1 : CLKIN
			//		2 : MultiSynth 0
			//		3 : My MultiSynth
			// 1:0	= CLK0 Output Rise and Fall time / Drive Strength Control.


		// Disable all clock
		m_i2c.write({ 0x0003, 0xFF });
		m_i2c.write({ 0x0010, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 });

		m_i2c.write({ 0x005A, 0x00, 0x00});	// MS6_P1, MS7_P1
		m_i2c.write({ 0x0095, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });	// 0x95 - 0x9B : Spread Spectrum
		m_i2c.write({ 0x00A2, 0x00, 0x00, 0x00 });	// 0xA2 - 0xA4 : VCXO
		m_i2c.write({ 0x00B7, (unsigned char)(0x12 |
			((m_nCrystalLoad <= 6) ?
				0x40 :
				(10 <= m_nCrystalLoad) ?
					0xC0 :
					0x80)) });

		m_i2c.write({ 0x0002, 0x53 });	// Interrupt Mask
		m_i2c.write({ 0x0007, 0x00 });
		m_i2c.write({ 0x000F, 0x00 });	// PLL config
			// 7:6	= CLKIN_DIV
			// 3	= PLLB_SRC
			// 2	= PLLA_SRC

		/* MSNA PLL section */
		m_i2c.write({ 0x001A,
			(uint8_t)(0xFF & (pll._P3 >> 8)),	// 7:0	= MSNA_P3[15:8]
			(uint8_t)(0xFF & (pll._P3 >> 0)),	// 7:0	= MSNA_P3[7:0]
			(uint8_t)(0x03 & (pll._P1 >> 16)),	// 1:0	= MSNA_P1[17:16]
			(uint8_t)(0xFF & (pll._P1 >> 8)),	// 7:0	= MSNA_P1[15:8]
			(uint8_t)(0xFF & (pll._P1 >> 0)),	// 7:0	= MSNA_P1[7:0]	
			(uint8_t)(
				(0xF0 & (pll._P3 >> 12)) |		// 7:4	= MSNA_P3[19:16]
				(0x0F & (pll._P2 >> 16))),		// 3:0	= MSNA_P2[19:16]
			(uint8_t)(0xFF & (pll._P2 >> 8)),	// 7:0	= MSNA_P2[15:8]
			(uint8_t)(0xFF & (pll._P2 >> 0)) });	// 7:0	= MSNA_P2[7:0]

		/* MS0 section */
		if (0 != clk0_fs)
		{
			m_i2c.write({ 0x002A,
				(uint8_t)(0xFF & (clk0._P3 >> 8)),		// 7:0 = MSx_P3[15:8]
				(uint8_t)(0xFF & (clk0._P3 >> 0)),		// 7:0 = MSx_P3[7:0]
				(uint8_t)(
					(0x70 & (clk0._Rdiv << 4)) |		// 6:4 = Rx_DIV[2:0]
					0 |									// 3:2 = MSx_DIVY4[1:0]
					(0x03 & (clk0._P1 >> 16)) ),			// 1:0 = MSx_P1[17:16]
				(uint8_t)(0xFF & (clk0._P1 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk0._P1 >> 0)),		// 7:0 = MSx_P1[7:0]
				(uint8_t)(
					(0xF0 & (clk0._P3 >> 12)) |			// 7:4 = MSx_P3[19:16]
					(0x0F & (clk0._P2 >> 16)) ),		// 3:0 = MSx_P2[19:16]
				(uint8_t)(0xFF & (clk0._P2 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk0._P2 >> 0)) });	// 7:0 = MSx_P1[7:0]
		}

		/* MS1 section */
		if (0 != clk1_fs)
		{
			m_i2c.write({ 0x0032,
				(uint8_t)(0xFF & (clk1._P3 >> 8)),		// 7:0 = MSx_P3[15:8]
				(uint8_t)(0xFF & (clk1._P3 >> 0)),		// 7:0 = MSx_P3[7:0]
				(uint8_t)(
					(0x70 & (clk1._Rdiv << 4)) |		// 6:4 = Rx_DIV[2:0]
					0 |									// 3:2 = MSx_DIVY4[1:0]
					(0x03 & (clk1._P1 >> 16)) ),		// 1:0 = MSx_P1[17:16]
				(uint8_t)(0xFF & (clk1._P1 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk1._P1 >> 0)),		// 7:0 = MSx_P1[7:0]
				(uint8_t)(
					(0xF0 & (clk1._P3 >> 12)) |			// 7:4 = MSx_P3[19:16]
					(0x0F & (clk1._P2 >> 16)) ),		// 3:0 = MSx_P2[19:16]
				(uint8_t)(0xFF & (clk1._P2 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk1._P2 >> 0)) });	// 7:0 = MSx_P1[7:0]
		}

		/* MS2 section */
		if (0 != clk2_fs)
		{
			m_i2c.write({ 0x003A,
				(uint8_t)(0xFF & (clk2._P3 >> 8)),		// 7:0 = MSx_P3[15:8]
				(uint8_t)(0xFF & (clk2._P3 >> 0)),		// 7:0 = MSx_P3[7:0]
				(uint8_t)(
					(0x70 & (clk2._Rdiv << 4)) |		// 6:4 = Rx_DIV[2:0]
					0 |									// 3:2 = MSx_DIVY4[1:0]
					(0x03 & (clk2._P1 >> 16)) ),		// 1:0 = MSx_P1[17:16]
				(uint8_t)(0xFF & (clk2._P1 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk2._P1 >> 0)),		// 7:0 = MSx_P1[7:0]
				(uint8_t)(
					(0xF0 & (clk2._P3 >> 12)) |			// 7:4 = MSx_P3[19:16]
					(0x0F & (clk2._P2 >> 16)) ),		// 3:0 = MSx_P2[19:16]
				(uint8_t)(0xFF & (clk2._P2 >> 8)),		// 7:0 = MSx_P1[15:8]
				(uint8_t)(0xFF & (clk2._P2 >> 0)) });	// 7:0 = MSx_P1[7:0]
		}

		/*  CLKx Control */
		m_i2c.write({ 0x0010, CLKxCTRL[0], CLKxCTRL[1], CLKxCTRL[2] });

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
		m_i2c.write({ 0x0010, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 });
	}

protected:
	static	uint32_t	GetLCM(uint32_t a, uint32_t b)
	{
		return	a * b / GetGCD(a, b);
	}

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