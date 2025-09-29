///===================================================================
/// File Name: GGG_GFX.h
/// Type     : C header
/// Purpose  : Algorithms for embedded graphics
/// Version  : 1.0
///===================================================================
/// Description:
///		This is a framebuffer implementation for embedded graphics.
///		The framebuffer is implemented as a 1D byte array.
///		Bit organization per byte is "LSB at top".
///		Our goal here is to fold a display matrix of COLMAX/ROWMAX
///		into the 1D frame buffer.
///
///		With a bitmap as input, the algorithm vertical-scans
///		its contents into the frame buffer.
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-SEP-24 / G.RUIZ
///     * This is an initial release
///===================================================================


#ifndef __GGG_GFX__
#define __GGG_GFX__


/// @attention Comment #undef to enable printf logging
#define DEBUG_LOG
#undef DEBUG_LOG


#include <stdint.h>

#ifdef DEBUG_LOG
#include <stdio.h>
#endif



///===================================================================
///			Sec_1_0 SPECIALIZED DEFINITIONS (Modify to fit)
///===================================================================

/// @attention STM32 CMSIS
#define STM32F103xB
#include "stm32f1xx.h"


/// @attention Definition for the OLED driver:
#define DRIVER_SH1106
//#define DRIVER_SSD1306


/// @attention Define dimensions for the OLED screen, in bits
#define OLED_SEGMENT_START_OFFSET	( 3U	)	/// CircuitRocks 1.3" SH1106 OLED
#define ROWMAX						( 32U	)	/// CircuitRocks 1.3" SH1106 OLED
#define COLMAX						( 132U	)	/// CircuitRocks 1.3" SH1106 OLED



/// Framebuffer page dimension, in bits
#define PAGEHEIGHT		( 8U	)
#define OLED_MAX_PAGE	( ROWMAX / PAGEHEIGHT	)


///===================================================================
///				Sec_2_0 DRIVER AND FONT INCLUDES
///===================================================================

/// Endianness of the bytes on the page
#define LSB_TOP

#ifdef DRIVER_SH1106
	#include "sh1106_def.h"
	#include "SH1106.h"

/* #elif DRIVER_SSD1306
/// To-do: Verified SSD1306 drivers...
	#include "ssd1306_def.h"
	#include "SSD1306.h"
	#undef LSB_TOP
*/
#endif

#include "font_4x6.h"
#include "font_5x7.h"



///===================================================================
///				Sec_3_0 TYPE DEFINITIONS AND CONSTANTS
///===================================================================

/// Size of the array storing the flattened matrix
#define FRAMESIZE	(( COLMAX * ROWMAX ) / PAGEHEIGHT )



///===================================================================
///					Sec_4_0 STRUCTURES AND ENUMS
///===================================================================



///===================================================================
///						Sec_5_0 FUNCTIONS
///===================================================================



///*******************************************************************

void gfx_draw_bitmap( uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t * bitmap, struct SH1106_Instance * display, uint8_t noload );
void gfx_print_string( uint16_t x, uint16_t y, const char * string, uint16_t size, uint8_t font_id, struct SH1106_Instance * display );
void gfx_set_px( uint16_t x, uint16_t y, uint8_t logic );
uint8_t gfx_get_px( uint16_t x, uint16_t y );
static uint16_t get_byte_index( uint16_t x, uint16_t y, uint16_t colmax );


#ifdef DEBUG_LOG
void gfx_console_print_framebuffer( void );
#endif


/// Select the bit on a vertical byte
#ifdef LSB_TOP
#define SET_PIXEL( y )		\
	( 0x01 << ( y & 0x07 ))
#else
#define SET_PIXEL( y )		\
	( 0x80 >> ( y & 0x07 ))
#endif


/// Divide-by-8 for MCU's with no division/floating point accelerators:
#define DIV_8( a )			\
	( a >> 3U )


/// Flattened matrix
uint8_t frame[FRAMESIZE] = { 0 };


static uint16_t get_byte_index( uint16_t x, uint16_t y, uint16_t colmax )
{
	return ( DIV_8( y ) * colmax ) + x;
}

void gfx_set_px( uint16_t x, uint16_t y, uint8_t logic )
{
	uint16_t bytepos = get_byte_index( x, y, COLMAX );

	if( logic > 0U )
		frame[bytepos] |= SET_PIXEL( y );
	else
		frame[bytepos] &= ~( SET_PIXEL( y ));
}

uint8_t gfx_get_px( uint16_t x, uint16_t y )
{
	uint16_t bytepos = get_byte_index( x, y, COLMAX );
	return frame[bytepos] & SET_PIXEL( y );
}

void gfx_draw_bitmap( uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t * bitmap, struct SH1106_Instance * display, uint8_t noload )
{
	uint8_t mask;
	uint16_t bitmap_byte_pos;
	uint16_t row_mod;

//	uint16_t x_origin = x;
	uint16_t y_origin = y;

	/// Vertical scan the bitmap:
	for( uint16_t col = 0U; col < width; col++ )
	{
		for( uint16_t row = 0U; row < height; row++ )
		{
			mask = 0x01 << ( row & ( PAGEHEIGHT - 1U ));

			/// Calculate the byte position on the bitmap. To prevent unneccessary
			/// repetition of get_byte_index() (which is 5 instruction cycles),
			/// do it once per byte-frame:
		//	row_mod = row & FRAMESIZE;	/// row % FRAMESIZE;
		//	if( row_mod == 0U )
				bitmap_byte_pos = get_byte_index( col, row, width );
	
			gfx_set_px( x, y, bitmap[bitmap_byte_pos] & mask );
			y++;
	
			/// If on the y-axis max, stop rendering this column
			if( y == ROWMAX )
				break;
		}

		y = y_origin;
		x++;

		/// If on the x-axix max, stop rendering this row
		if( x == COLMAX )
			break;
	}

#ifdef DRIVER_SH1106
	/// Not capable of auto-incrementing page address. We loop across the pages...
	/// To do: Add an algorithm that will only update modified pages instead of the whole framebuffer...
	if( !noload )
		SH1106_Write_Frame_Buffer( display, 0x00UL, OLED_MAX_PAGE, 0U, COLMAX, COLMAX, frame );

/* #else // DRIVER_SSD1306
	/// Capable of auto-incrementing page address on horizontal-scan mode...
*/
#endif	
}

/// Returns index of the glyph for the given character c
/// We dont transfer font data into an intermediate structure
/// to save stack space.
static uint8_t check_font_lookup( const char c, uint8_t font_id )
{
	uint8_t depth;


	if( font_id == font_5x7_id )
	{
		depth = FONT_5X7_LUT_DEPTH;
		for( uint8_t i = 0U; i < depth; i++ )
		{
			if( c == font_5x7_lookup[i] )
				return i;
		}
	}
	else
	{
		depth = FONT_4X6_LUT_DEPTH;
		for( uint8_t i = 0U; i < depth; i++ )
		{
			if( c == font_4x6_lookup[i] )
				return i;
		}
	}

	/// If character is not found, return <space> glyph index
	return 0U;
}

/// @brief Updates the framebuffer with a text string. Uploads the whole framebuffer afterwards.
/// @param x		x-origin
/// @param y 		y-origin
/// @param string 	Array of characters
/// @param size		Size of the character array
/// @param font_id	Font ID. Check the .h files
/// @param display	SH1106_Instance of the display
void gfx_print_string( uint16_t x, uint16_t y, const char * string, uint16_t size, uint8_t font_id, struct SH1106_Instance * display )
{
	uint8_t glyph_index, width, height;
	uint16_t x_ptr = 0U;


	if( font_id == font_5x7_id )
	{
		width = font_5x7_width;
		height = font_5x7_height;
	}
	else
	{
		width = font_4x6_width;
		height = font_4x6_height;
	}

	for( uint16_t i = 0U; i < size; i++ )
	{
		glyph_index = check_font_lookup( string[i], font_id );
		x_ptr = x + (i * width);


		if( font_id == font_5x7_id )
			gfx_draw_bitmap( x_ptr, y, width, height, font_5x7_glyphtable[glyph_index], display, 0x01UL );
		else
			gfx_draw_bitmap( x_ptr, y, width, height, font_4x6_glyphtable[glyph_index], display, 0x01UL );

	}
#ifdef DRIVER_SH1106
	SH1106_Write_Frame_Buffer( display, 0x00UL, OLED_MAX_PAGE, 0U, COLMAX, COLMAX, frame );
#endif
}


#ifdef DEBUG_LOG
void gfx_console_print_framebuffer( void )
{
	uint8_t px;

	/// Perform horizontal scanning for the print, since we dgaf how slow it'll be
	for( uint16_t y = 0U; y < ROWMAX; y++ )
	{
		for( uint16_t x = 0U; x < COLMAX; x++ )
		{
			px = gfx_get_px( x, y );

			if( px )
				printf( "1 " );
			else
				printf( "0 " );
		}
		printf( "\n" );
	}
}
#endif




#endif /*__GGG_GFX__*/