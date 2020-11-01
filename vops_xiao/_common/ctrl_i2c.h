#ifndef __CTRL_I2C_H_INCLUDED__
#define __CTRL_I2C_H_INCLUDED__

class ctrl_i2c
{
public:
	ctrl_i2c( int addr ) : m_addr(addr)
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
	const int     m_addr;
};

#endif
