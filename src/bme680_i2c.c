///===================================================================
/// File Name: bme680_i2c.c
/// Type     : STM32 C-header
/// Purpose  : BME680 Driver
/// Version  : 1.0
///===================================================================
/// Description
///     * Driver for Bosch BME860 sensor
///     * Gas, pressure, temperature, humidity sensor
///     * I2C communication driver
///		* Modify sections SEC_1_0 and SEC_2_0 according to SDO strap
///		configuration and target MCU I2C driver file, respectively
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
///===================================================================



#include "bme680_def.h"
#include "bme680_i2c.h"


#ifdef SDO_TO_VDDIO
#define BME680_I2C_ID ( BME680_I2C_ADDR_VDDIO << 1U )
#else
#define BME680_I2C_ID ( BME680_I2C_ADDR_GND << 1U )
#endif


///===================================================================
///						BME680 I2C functions
///===================================================================

/// @brief  Initialize chosen I2C channel
/// @param mcu_clk_mhz MCU clock frequency (e.g. 8MHz = 0x08UL)
/// @param mcu_period_ns MCU clock period (e.g. 8MHz = 125 ns )
/// @param i2c_sel I2C1 = 0x01UL, I2C2 = 0x02UL
void bme680_i2c_initialize( uint32_t mcu_clk_mhz, uint32_t mcu_period_ns, uint8_t i2c_sel )
{
	I2C_INIT( mcu_clk_mhz, mcu_period_ns, i2c_sel, I2C_FM_400KHZ );
}

/// @brief Write array of address-data pairs
/// @param i2c_sel I2C1 = 0x01UL, I2C2 = 0x02UL
/// @param count Number of bytes to write
/// @param wrbyte Pass-by-reference pointer to an array of bytes to write
void bme680_i2c_write( uint8_t i2c_sel, uint8_t count, uint8_t * wrbyte )
{
	I2C_START( i2c_sel );
	I2C_WR_DEVICE_ID( i2c_sel, BME680_I2C_ID );
	I2C_WRITE( i2c_sel, count, wrbyte );
	I2C_STOP( i2c_sel );
}

/// @brief Read and store to a byte array
/// @param i2c_sel I2C1 = 0x01UL, I2C2 = 0x02UL
/// @param addr Start address (refer to BME680 memory map)
/// @param count Number of bytes to write
/// @param rdbyte Pass-by-reference pointer to byte-array storage
void bme680_i2c_read( uint8_t i2c_sel, uint8_t addr, uint8_t count, uint8_t * rdbyte )
{
	I2C_START( i2c_sel );
	I2C_WR_DEVICE_ID( i2c_sel, BME680_I2C_ID );
	I2C_WRITE( i2c_sel, 0x01UL, &addr );
	I2C_RESTART( i2c_sel );
	I2C_READ( i2c_sel, ( BME680_I2C_ID | 0x01UL ), count, rdbyte );
}