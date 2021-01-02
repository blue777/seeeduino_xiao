#ifndef __CTRL_I2C_H_INCLUDED__
#define __CTRL_I2C_H_INCLUDED__

#include <cstdint>
#include <initializer_list>

class ctrl_i2c
{
public:
	ctrl_i2c( uint8_t addr ) : m_addr(addr)
	{
	}

	bool    write( const unsigned char * data, int size )
	{
		Wire.beginTransmission(m_addr);
		for ( int i = 0; i < size; i++ )
		{
			Wire.write( data[i] );
		}
		Wire.endTransmission();
		return  true;
	}

	bool	write(std::initializer_list<const unsigned char> data)
	{
		return	write( data.begin(), data.size() );
	}

	bool    read( unsigned char * data, int size )
	{
		int i = 0;

		Wire.requestFrom( m_addr, size );
		while ( Wire.available() )
		{
			data[i++] = Wire.read();
		}
		return  true;
	}

private:
	const uint8_t     m_addr;
};

#endif
