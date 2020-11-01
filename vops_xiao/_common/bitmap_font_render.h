#pragma once

#include <stdint.h>

typedef struct tagCHAR_INFO
{
	int	nFontWidth;
	int	nFontHeight;
	const uint8_t * data;
} tagCHAR_INFO;

typedef struct tagBITMAP_FONT
{
	int	nFontHeight;
	tagCHAR_INFO	tInfo[128];
} tagBITMAP_FONT;



void	BitmapFont_CalcRect(const tagBITMAP_FONT& tFont, const char * pszString, int &width, int &height)
{
	int		pos_x = 0;
	int		pos_y = 0;

	width = 0;
	height = 0;

	for (;*pszString != '\0'; pszString++)
	{
		pos_x += tFont.tInfo[*pszString].nFontWidth;

		if (width < pos_x)
		{
			width = pos_x;
		}
		if (height < (pos_y + tFont.nFontHeight))
		{
			height = pos_y + tFont.nFontHeight;
		}

		switch (*pszString)
		{
		case '\n':
			pos_y += tFont.nFontHeight;
			pos_x = 0;
			break;
		}
	}
}

template<class PIXEL>
void	BitmapFont_DrawText(const tagBITMAP_FONT& tFont, PIXEL* image, int stride, int width, int height, int pos_x, int pos_y, const char *pszString, PIXEL color=0xFFFFFFFF )
{
	int	start_x = pos_x;

	for(;*pszString != '\0'; pszString++)
	{
		switch (*pszString)
		{
		case '\n':
			pos_x = start_x;
			pos_y += tFont.nFontHeight;
			break;

		case '\t':
			break;

		default:
			{
				const tagCHAR_INFO&	tInfo = tFont.tInfo[*pszString];
				int	tx = 0;
				int	ty = 0;
				int	tw = tInfo.nFontWidth;
				int	th = tInfo.nFontHeight;

				if (pos_x < 0)
				{
					tw += pos_x;
					tx -= pos_x;
				}

				if (width < (pos_x + tInfo.nFontWidth) )
				{
					tw -= pos_x + tInfo.nFontWidth - width;
				}

				if (pos_y < 0)
				{
					th += pos_y;
					ty -= pos_y;
				}

				if (height < (pos_y + tInfo.nFontHeight))
				{
					th -= pos_y + tInfo.nFontHeight - height;
				}

				if ((0 < tw) && (0 < th))
				{
					for (int y = 0; y < th; y++)
					{
						PIXEL*	dst = (PIXEL*)( ((uint8_t*)image) + stride * (pos_y + ty + y) );

						for (int x = 0; x < tw; x++)
						{
							uint8_t	bit = (tInfo.data[tInfo.nFontWidth * ((ty+y) / 8) + tx + x] >> ((ty+y) & 7)) & 1;

							if (bit)
							{
								dst[pos_x+tx+x] = color;
							}
						}
					}
				}

				pos_x += tInfo.nFontWidth;
			}
			break;
		}
	}
}
