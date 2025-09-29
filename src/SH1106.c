///===================================================================
/// File Name: SH1106.c
/// Type     : STM32 C-file
/// Purpose  : SH1106 Low-level Driver
/// Version  : 1.0
///===================================================================
/// Description
///     * Driver for SH1106 OLED Display
///     * I2C communication driver
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Sep-19 / G.RUIZ
///		* Initial release
///===================================================================


#include <stdint.h>

#include "sh1106_def.h"
#include "ggg_sh1106_i2c.h"
#include "SH1106.h"




///******************* Function Declarations *************************
void SH1106_Initialize( struct SH1106_Instance * display );
void SH1106_Set_Contrast( struct SH1106_Instance * display );
void SH1106_Set_EntireDisplay_Mode( struct SH1106_Instance * display );
void SH1106_Set_Display_Logic( struct SH1106_Instance * display );
void SH1106_Set_Display( struct SH1106_Instance * display );
void SH1106_Set_Address( struct SH1106_Instance * display, uint8_t col_addr, uint8_t page_addr );
void SH1106_Write_Page_Data( struct SH1106_Instance * display, uint8_t page, uint8_t col_start, uint16_t size, uint8_t * array );
void SH1106_Clear_DisplayRAM( struct SH1106_Instance * display );
void SH1106_Sample_Setup( struct SH1106_Instance * display );

static void SH1106_HW_Config( struct SH1106_Instance * display );
static void SH1106_Set_Timing_and_Driving_Scheme( struct SH1106_Instance * display );
static uint8_t assemble_ctrl_byte( ContBitType co_bit, DataCmdBitType dc_bit );


///******************** Function Definitions *************************


/// @brief Initial I2C, hardware, timing and driving, display settings.
/// Will also clear RAM contents, then turns the display on.
/// Make sure to fill up the SH1106_HWSetup and SH1106_TimingDriveSetup config
/// before calling this function.
/// @param display Pass-by-reference pointer to SH1106 display object
void SH1106_Initialize( struct SH1106_Instance * display )
{
	sh1106_i2c_initialize( MCU_FREQUENCY_MHZ, MCU_PERIOD_NS, display->i2c_channel );

	SH1106_HW_Config( display );							/// Hardware config
	SH1106_Set_Contrast( display );							/// Contrast
	SH1106_Set_EntireDisplay_Mode( display );				/// Entire Display Mode
	SH1106_Set_Display_Logic( display );					/// Display logic
	SH1106_Set_Timing_and_Driving_Scheme( display );		/// Timing and drive scheme config
	SH1106_Clear_DisplayRAM( display );						/// Set RAM contents to 0x00

	display->enable = DISPLAY_ON;	
	SH1106_Set_Display( display );							/// Set Display On
}

static void SH1106_HW_Config( struct SH1106_Instance * display )
{
	uint8_t _wrbyte[9];
	uint8_t index = 0U;

	/// Set mux ratio
	_wrbyte[index++] = SH1106_CMD_SET_MUX_RATIO;
	_wrbyte[index++] = display->hwsetup.mux_ratio & SH1106_CMD_MUX_RATIO_Mask;

	/// COM display offset
	_wrbyte[index++] = SH1106_CMD_SET_DISP_OFFSET;
	_wrbyte[index++] = display->hwsetup.disp_offset & SH1106_CMD_DISP_OFFSET_Mask;

	/// Display start line
	_wrbyte[index++] = SH1106_CMD_SET_DISPLAY_START_LINE | ( display->hwsetup.disp_start_line & SH1106_CMD_DISPLAY_START_LINE_Mask );

	/// Set segment remap
	_wrbyte[index++] = SH1106_CMD_SET_SEGMENT_REMAP | ( display->hwsetup.seg_remap & SH1106_CMD_SEGMENT_REMAP_Mask );

	/// COM output scan direction
	_wrbyte[index++] = SH1106_CMD_SET_COM_SCAN_DIR | (( display->hwsetup.com_scan_dir << SH1106_CMD_COM_SCAN_DIR_Pos ) & SH1106_CMD_COM_SCAN_DIR_Mask );

	/// COM pad hw config
	_wrbyte[index++] = SH1106_CMD_SET_PAD_HW_CONFIG;
	_wrbyte[index++] = SH1106_CMD_SEQ_ALT_MODE | (( display->hwsetup.pin_cfg << SH1106_CMD_SEQ_ALT_MODE_Pos ) & SH1106_CMD_SEQ_ALT_MODE_Mask );

	sh1106_i2c_write( display->i2c_channel, index, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_CMD_BYTE ), _wrbyte );
}

static void SH1106_Set_Timing_and_Driving_Scheme( struct SH1106_Instance * display )
{
	uint8_t _wrbyte[7];
	uint8_t index = 0U;

	/// Set Osc freq and Divide ratio
	_wrbyte[index++] = SH1106_CMD_SET_CLK_RATIO_FREQ;
	_wrbyte[index++] = (( display->tdsetup.osc_freq << SH1106_CMD_CLK_FREQ_Pos ) & SH1106_CMD_CLK_FREQ_Mask ) |
						( display->tdsetup.div_ratio & SH1106_CMD_CLK_RATIO_Mask );

	/// Charge periods
	_wrbyte[index++] = SH1106_CMD_SET_CHARGE_PERIOD;
	_wrbyte[index++] = (( display->tdsetup.discharge << SH1106_CMD_DISCHARGE_PERIOD_Pos ) & SH1106_CMD_DISCHARGE_PERIOD_Mask ) |
						( display->tdsetup.precharge & SH1106_CMD_PRECHARGE_PERIOD_Mask );

	/// Set VCOM desel
	_wrbyte[index++] = SH1106_CMD_SET_VCOM_DESEL_LVL;
	_wrbyte[index++] = display->tdsetup.vcom_desel & SH1106_CMD_VCOM_DESEL_LVL_Mask;

	/// Charge pump voltage
	_wrbyte[index++] = SH1106_CMD_SET_CHARGEPUMP_VOLTAGE | ( display->tdsetup.cpvoltage & SH1106_CMD_CHARGEPUMP_VOLTAGE_Mask );

	sh1106_i2c_write( display->i2c_channel, index, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_CMD_BYTE ), _wrbyte );
}

void SH1106_Set_Contrast( struct SH1106_Instance * display )
{
	uint8_t _wrbyte[2];

	_wrbyte[0] = SH1106_CMD_SET_CONTRAST;
	_wrbyte[1] = display->contrast & SH1106_CMD_CONTRAST_Mask;

	sh1106_i2c_write( display->i2c_channel, 2U, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_CMD_BYTE ), _wrbyte );
}

void SH1106_Set_EntireDisplay_Mode( struct SH1106_Instance * display )
{
	uint8_t _wrbyte = SH1106_CMD_SET_ENTIRE_DISPLAY_MODE | ( display->entire_display_mode & SH1106_CMD_ENTIRE_DISPLAY_MODE_Mask );
	sh1106_i2c_write( display->i2c_channel, 1U, assemble_ctrl_byte( CONT_BIT_SINGLE_BYTE, DCN_BIT_CMD_BYTE ), &_wrbyte );
}

void SH1106_Set_Display_Logic( struct SH1106_Instance * display )
{
	uint8_t _wrbyte = SH1106_CMD_SET_DISPLAY_LOGIC | ( display->logic & SH1106_CMD_DISPLAY_LOGIC_Mask );
	sh1106_i2c_write( display->i2c_channel, 1U, assemble_ctrl_byte( CONT_BIT_SINGLE_BYTE, DCN_BIT_CMD_BYTE ), &_wrbyte );
}

/// @brief Enables or disables DC-DC and Display. Updates SH1106_Instance.enable member.
/// @param display Pass-by-reference pointer to SH1106 display object
void SH1106_Set_Display( struct SH1106_Instance * display )
{
	uint8_t _wrbyte[3];
	uint8_t index = 0U;

	if( display->enable == DISPLAY_ON )
	{
		/// DC-DC on
		_wrbyte[index++] = SH1106_CMD_SET_DC_CTRL;
		_wrbyte[index++] = SH1106_CMD_SET_DC_MODE | SH1106_CMD_DC_MODE_Mask;

		/// Display on
		_wrbyte[index++] = SH1106_CMD_SET_DISPLAY_CTRL | SH1106_CMD_DISPLAY_CTRL_Mask;
	}
	else /*DISPLAY_OFF*/
	{
		/// DC-DC off
		_wrbyte[index++] = SH1106_CMD_SET_DC_CTRL;
		_wrbyte[index++] = SH1106_CMD_SET_DC_MODE & ~SH1106_CMD_DC_MODE_Mask;

		/// Display off
		_wrbyte[index++] = SH1106_CMD_SET_DISPLAY_CTRL & ~SH1106_CMD_DISPLAY_CTRL_Mask;
	}

	sh1106_i2c_write( display->i2c_channel, index, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_CMD_BYTE ), _wrbyte );	
}

/// @brief Set addressing mode. Note that column address auto-increments, but not page address.
/// @param display Pass-by-reference pointer to SH1106 display object
/// @param col_addr Column start address
/// @param page_addr Page address
void SH1106_Set_Address( struct SH1106_Instance * display, uint8_t col_addr, uint8_t page_addr )
{
	uint8_t _wrbyte[3];
	uint8_t index = 0U;

	/// Gate max column address
	if( col_addr > SH1106_COL_ADDR_MAX )
		col_addr = SH1106_COL_ADDR_MAX;

	/// Set column high-nibble
	_wrbyte[index++] = SH1106_CMD_SET_COL_HI_ADDR | (( col_addr & SH1106_CMD_COL_HI_Mask ) >> 4U );
	
	/// Set column low-nibble
	_wrbyte[index++] = col_addr & SH1106_CMD_COL_LO_Mask;

	/// Set page address
	_wrbyte[index++] = SH1106_CMD_SET_PAGE_ADDR | ( page_addr & SH1106_CMD_PAGE_ADDR_Mask );

	sh1106_i2c_write( display->i2c_channel, index, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_CMD_BYTE ), _wrbyte );	
}

/// @brief Write the framebuffer to Display RAM.
/// @param display		Pass-by-reference pointer to SH1106 display object
/// @param page_start	Page start address
/// @param page_stop	Page stop address (+ 1)
/// @param col_start	Column start address
/// @param col_start	Column stop address
/// @param frame_size	constant framebuffer size
/// @param framebuffer	Pass-by-reference pointer to data array
void SH1106_Write_Frame_Buffer( struct SH1106_Instance * display, uint8_t page_start, uint8_t page_stop,
							uint8_t col_start, uint8_t col_stop, uint16_t page_size, uint8_t * framebuffer )
{
	uint16_t ptr;

	for( uint8_t p = page_start; p < page_stop; p++ )
	{
		SH1106_Set_Address( display, col_start, p );

		/// Write ctrl byte with i2c start sequence
		sh1106_i2c_write_this_byte( display->i2c_channel, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_DATA_BYTE ), 0x01UL, 0x00UL );

		for( uint16_t c = col_start; c <= col_stop; c++ )	/// we do this col_start/stop in case we upgrade this shit in the future...
		{
			ptr = ( p * page_size ) + c;
			sh1106_i2c_write_this_byte( display->i2c_channel, framebuffer[ptr], 0x00UL, ( c == col_stop ) ? 0x01UL : 0x00UL );
		}
	}
}

void SH1106_Clear_DisplayRAM( struct SH1106_Instance * display )
{
	uint8_t _zeroes[131];

	for( uint8_t seg = 0U; seg < SH1106_COL_ADDR_MAX; seg++ )
	{
		_zeroes[seg] = 0x00UL;
	}

	for( uint8_t page = 0U; page < SH1106_PAGE_ADDR_MAX; page++ )
	{
		SH1106_Set_Address( display, 0x00UL, page );
		sh1106_i2c_write( display->i2c_channel, SH1106_COL_ADDR_MAX, assemble_ctrl_byte( CONT_BIT_STREAM_BYTES, DCN_BIT_DATA_BYTE ),_zeroes );	
	}
}

static uint8_t assemble_ctrl_byte( ContBitType co_bit, DataCmdBitType dc_bit )
{
	return (uint8_t)(( co_bit << SH1106_CTRL_BYTE_CO_Pos ) | ( dc_bit << SH1106_CTRL_BYTE_DC_Pos ));
}

void SH1106_Sample_Setup( struct SH1106_Instance * display )
{
	display->i2c_channel = 0x01UL; /// I2C1
	display->entire_display_mode = ENTIRE_DISPLAY_NORMAL;
	display->logic = DISPLAY_LOGIC_NORMAL;
	display->contrast = 0x7FUL;

	display->hwsetup.disp_start_line = 0x00UL;
	display->hwsetup.seg_remap = SEG_REMAP_NORMAL;
	display->hwsetup.mux_ratio = 0x3FUL;
	display->hwsetup.com_scan_dir = COM_SCAN_NORMAL;
	display->hwsetup.disp_offset = 0x00UL;
	display->hwsetup.pin_cfg = COM_PIN_SEQUENTIAL;

	display->tdsetup.osc_freq = 0x08UL;
	display->tdsetup.div_ratio = 0x00UL;
	display->tdsetup.vcom_desel = 0x03UL;
	display->tdsetup.precharge = 0x02UL;
	display->tdsetup.discharge = 0x02UL;
	display->tdsetup.cpvoltage = CHARGEPUMP_8V0;
}