///===================================================================
/// File Name: ggg_sh1106_i2c.c
/// Type     : STM32 C-file
/// Purpose  : SH1106 I2C Driver
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

#define SA0_TO_GND

#ifdef SA0_TO_GND
#define SH1106_I2C_ID		( SH1106_I2C_ADDR_GND << 1U )
#else
#define SH1106_I2C_ID		( SH1106_I2C_ADDR_VCC << 1U )
#endif



void sh1106_i2c_initialize( uint32_t mcu_clk_mhz, uint32_t mcu_period_ns, uint8_t i2c_sel )
{
	I2C_INIT( mcu_clk_mhz, mcu_period_ns, i2c_sel, I2C_FM_400KHZ );
}


/// @brief Perform I2C-write by writing a ctrl byte, then the byte/byte-array
/// @param i2c_sel i2c channel
/// @param count size of the byte-array
/// @param ctrlbyte control byte required by the display device
/// @param wrbyte pass-by-reference pointer to the byte or byte array
void sh1106_i2c_write( uint8_t i2c_sel, uint16_t count, uint8_t ctrlbyte, uint8_t * wrbyte )
{
	I2C_START( i2c_sel );
	I2C_WR_DEVICE_ID( i2c_sel, SH1106_I2C_ID );
	I2C_WRITE( i2c_sel, 1U, &ctrlbyte );
	I2C_WRITE( i2c_sel, count, wrbyte );
	I2C_STOP( i2c_sel );
}

void sh1106_i2c_write_this_byte( uint8_t i2c_sel, uint8_t wrbyte, uint8_t start_seq, uint8_t stop_seq )
{
	if( start_seq )
	{
		I2C_START( i2c_sel );
		I2C_WR_DEVICE_ID( i2c_sel, SH1106_I2C_ID );
	}

	I2C_WRITE( i2c_sel, 1U, &wrbyte );

	if( stop_seq )
		I2C_STOP( i2c_sel );
}
