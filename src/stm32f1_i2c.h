///===================================================================
/// File Name: stm32f1_i2c.h
/// Type     : STM32 C-header
/// Purpose  : I2C - STM32F103RB Nucleo
/// Version  : 1.1
///===================================================================
/// Description
///		* I2C control for STM32F103RB
///		* Based on RM0008 reference manual
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
/// Version/Date : v1.1 / 2025-Sep-03 / G.RUIZ
///		* Added STM32F1 prefix
///		* Replaced argument datatypes with stdint.h datatypes
///		* Added option to change MCU frequency
///		* Prepped functions to return error codes
///		* Renamed to stm32f1_i2c
///===================================================================



#ifndef STM32F1_I2C_H_
#define STM32F1_I2C_H_

#include <stdint.h>

typedef enum
{
	STM32F1_I2C_OFF,
	STM32F1_I2C_CH1,
	STM32F1_I2C_CH2
} STM32F1_I2C_CH;


typedef enum
{
	I2C_SM_100KHZ,
	I2C_FM_400KHZ
} I2C_Mode;


typedef enum
{
	I2C_CMD_EXECUTED,
	I2C_CMD_SKIPPED,
	I2C_ERR_NOT_INITIALIZED,
	I2C_ERR_START_FAILED,
	I2C_ERR_DEVID_NACK,
	I2C_ERR_TXE_TIMEOUT,
	I2C_ERR_BTF_TIMEOUT,
	I2C_ERR_RXNE_TIMEOUT,
	I2C_ERR_OTHER
} I2C_Error_Code;

extern void stm32f1_i2c_init(uint32_t mcu_freq_mhz, uint32_t mcu_period_ns, uint8_t i2csel, I2C_Mode i2cspeed);
extern void stm32f1_i2c_start(uint8_t i2csel);
extern void stm32f1_i2c_restart(uint8_t i2csel);
extern void stm32f1_i2c_stop(uint8_t i2csel);
extern void stm32f1_i2c_send_slave_id(uint8_t i2csel, uint8_t devid);
extern void stm32f1_i2c_write(uint8_t i2csel, uint32_t bytecount, uint8_t * wrbyte);
extern void stm32f1_i2c_read(uint8_t i2csel, uint8_t devid, uint32_t bytecount, uint8_t * rdbyte);



#endif ///STM32F1_I2C_H_