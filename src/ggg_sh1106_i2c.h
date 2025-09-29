///===================================================================
/// File Name: ggg_sh1106_i2c.h
/// Type     : STM32 C-header
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



#ifndef GGG_SH1106_I2C__
#define GGG_SH1106_I2C__


/// @attention Replace with the target MCU's I2C driver
#include "stm32f1_i2c.h"


///	@attention Replace the definitions with target MCU's I2C functions

#define I2C_INIT( clkmhz, clkns, ch, mode )		\
	stm32f1_i2c_init( clkmhz, clkns, ch, mode )

#define I2C_START( ch )							\
	stm32f1_i2c_start( ch )

#define I2C_RESTART( ch )						\
	stm32f1_i2c_restart( ch )

#define I2C_STOP( ch )							\
	stm32f1_i2c_stop( ch )

#define I2C_WR_DEVICE_ID( ch, devid )			\
	stm32f1_i2c_send_slave_id( ch, devid )

#define I2C_WRITE( ch, count, arr )				\
	stm32f1_i2c_write( ch, count, arr )

//#define I2C_FRAME_WRITE( ch, pstart, pstop, cstart, cstop, fsize, arr )
//	stm32f1_i2c_frame_section_write( ch, pstart, pstop, cstart, cstop, fsize, arr )


//#define I2C_READ( ch, devid, count, arr )		\
//	stm32f1_i2c_read( ch, devid, count, arr )


/// @brief comm functions
extern void sh1106_i2c_initialize( uint32_t mcu_clk_mhz, uint32_t mcu_period_ns, uint8_t i2c_sel );
extern void sh1106_i2c_write( uint8_t i2c_sel, uint16_t count, uint8_t ctrlbyte, uint8_t * wrbyte );
extern void sh1106_i2c_write_this_byte( uint8_t i2c_sel, uint8_t wrbyte, uint8_t start_seq, uint8_t stop_seq );

//extern void ssd1306_i2c_read(uint8_t i2c_sel, uint8_t addr, uint8_t count, uint8_t * rdbyte);



#endif /*GGG_SH1106_I2C__*/