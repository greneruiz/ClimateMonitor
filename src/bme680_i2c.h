///===================================================================
/// File Name: bme680_i2c.h
/// Type     : STM32 C-header
/// Purpose  : BME680 Driver
/// Version  : 1.0
///===================================================================
/// Description
///     * Driver for Bosch BME860 sensor
///     * Gas, pressure, temperature, humidity sensor
///     * I2C communication driver
///		* Modify sections according to SDO strap
///		configuration and target MCU I2C driver file, respectively
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
///===================================================================




#ifndef BME680_I2C_H_
#define BME680_I2C_H_


/// @attention Undefine if SDO is connected to GND
#define SDO_TO_VDDIO
//#undef SDO_TO_VDDIO

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

#define I2C_READ( ch, devid, count, arr )		\
	stm32f1_i2c_read( ch, devid, count, arr )


/// @brief BME680 comm functions
extern void bme680_i2c_initialize(uint32_t mcu_clk_mhz, uint32_t mcu_period_ns, uint8_t i2c_sel);
extern void bme680_i2c_write(uint8_t i2c_sel, uint8_t count, uint8_t * wrbyte);
extern void bme680_i2c_read(uint8_t i2c_sel, uint8_t addr, uint8_t count, uint8_t * rdbyte);




#endif ///BME680_I2C_H_