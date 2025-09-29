///===================================================================
/// File Name: stm32f1_i2c.c
/// Type     : STM32 C-source
/// Purpose  : I2C - STM32F103RB Nucleo
/// Version  : 1.1
///===================================================================
/// Description
///		* I2C control for STM32F103RB
///		* Based on RM0008 reference manual
///		* Threadlock on status polling
///		* To-Do:
///			- No threadlocking
///			- Pin/speed remapping
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
/// Version/Date : v1.1 / 2025-Sep-03 / G.RUIZ
///		* Added STM32F1 prefix
///		* Replaced argument datatypes with stdint.h datatypes
///		* Added option to change MCU frequency
///		* Renamed to stm32f1_i2c
///===================================================================


/// SEC_1.0 STM32 definition *****************************************

#include <stdint.h>

#define STM32F103xB
#include "stm32f1xx.h"
#include "stm32f1_i2c.h"


/// SEC_2.0 Constants ************************************************



/// I2C Ready Mask
#define I2C1_RDY		( 0x01UL		)
#define I2C2_RDY		( 0x02UL		)
#define I2C_ALL_RDY		( 0x03UL		)

#define FM_TRE_NS		( 300U			)
#define FM_PERIOD_NS	( 2500U			)
#define SM_TRE_NS		( 1000U			)
#define SM_PERIOD_NS	( 10000U		)

/// I2C SCL timeout multiplier for wait times
#define TIMEOUT_MULT	( 8U			)
#define TIMEOUT_LIMIT	( 20U			)
/// Slave ID mask
#define DEVID_MODE_MASK	( 0x0EUL		)




/// SEC_3.0 Function definitions *************************************

#define SELECT_I2C_CHAN( ch )		\
	I2C_TypeDef * _i2csel = ch == STM32F1_I2C_CH1 ? I2C1 : I2C2


static void stm32f1_i2c_config_and_enable( I2C_TypeDef * i2csel, uint32_t mcu_freq_mhz, uint32_t mcu_period_ns, I2C_Mode i2cspeed );
static uint8_t wait_timeout( I2C_TypeDef * i2csel, uint16_t statusbit, I2C_Error_Code errcode );


/// @brief Initialize selected I2C (For F103xB: I2C1 or I2C2) using I2C_TypeDef* datatype
/// @param i2c_sel : I2C1 or I2C2 for STM32F1xx
void stm32f1_i2c_init( uint32_t mcu_freq_mhz, uint32_t mcu_period_ns, uint8_t i2csel, I2C_Mode i2cspeed )
{
	/// GPIOB & Alt Function clock enable : addr 0x40021018
	MODIFY_REG( RCC->APB2ENR, ( RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN ), ( RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN ));
	SELECT_I2C_CHAN( i2csel );

	if( i2csel == STM32F1_I2C_CH1 )								/// I2C1 is not ready and I2C1 is being initialized
	{
		//i2c_initialized_mask |= 0x01;
		/// Configure GPIO: Output 10MHz, Open-drain :
		MODIFY_REG( GPIOB->CRL, ( GPIO_CRL_CNF6_Msk | GPIO_CRL_MODE6_Msk ), ( GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0 ));
		MODIFY_REG( GPIOB->CRL, ( GPIO_CRL_CNF7_Msk | GPIO_CRL_MODE7_Msk ), ( GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0 ));
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C1EN );			/// I2C1 clock enable
	}
	else if( i2csel == STM32F1_I2C_CH2 )
	{
		//i2c_initialized_mask |= 0x02;
		/// Configure GPIO: Output 10MHz, Open-drain :
		MODIFY_REG( GPIOB->CRH, ( GPIO_CRH_CNF10_Msk | GPIO_CRH_MODE10_Msk ), ( GPIO_CRH_CNF10 | GPIO_CRH_MODE10_0 ));
		MODIFY_REG( GPIOB->CRH, ( GPIO_CRH_CNF11_Msk | GPIO_CRH_MODE11_Msk ), ( GPIO_CRH_CNF11 | GPIO_CRH_MODE11_0 ));
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_I2C2EN );			/// I2C2 clock enable
	}

	stm32f1_i2c_config_and_enable(_i2csel, mcu_freq_mhz, mcu_period_ns, i2cspeed );
}

/// @brief Wait for Idle, perform I2C start sequence, then wait for SB status bit
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
void stm32f1_i2c_start( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	while( READ_BIT(_i2csel->SR2, I2C_SR2_BUSY )){}				/// Check BUSY status *
	SET_BIT(_i2csel->CR1, I2C_CR1_START );						/// Generate start
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_SB )){}				/// Check Start sequence *
}

/// @brief Send another start sequence
/// @brief Dummy-read RD register then resend start sequence
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
void stm32f1_i2c_restart( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	READ_REG(_i2csel->DR );										/// Dummy-read DR to clear BTF status bit
	SET_BIT(_i2csel->CR1, I2C_CR1_START );						/// Generate restart
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_SB )){}				/// Check Start sequence *
}

/// @brief Perform I2C stop sequence
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
void stm32f1_i2c_stop( uint8_t i2csel )
{
	SELECT_I2C_CHAN( i2csel );
	SET_BIT(_i2csel->CR1, I2C_CR1_STOP );
}

/// @brief Used after i2c_start(), send device slave ID.
/// @brief Write slave ID, wait for ADDR status bit, then clear it by reading SR2 register
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
/// @param devid : 1-byte device ID with LSB as Rd/Wr bit
void stm32f1_i2c_send_slave_id( uint8_t i2csel, uint8_t devid )
{
	SELECT_I2C_CHAN( i2csel );
	WRITE_REG(_i2csel->DR, devid );								/// Write device id 
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}			/// Wait until AddrStatus is asserted
	READ_REG(_i2csel->SR2 );									/// Read SR2 to clear SR1[ADDR] bit
}

/// @brief Used after i2c_send_slave_id(), send byte/s from a byte array.
/// @brief Wait for ADDR transmit; write byte/s, then wait for completion; after last byte, wait for BTF status bit
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
/// @param bytecount : Number of bytes to write (min: 1); must match wrbyte[] size
/// @param wrbyte : Pass-by-reference pointer to byte array
void stm32f1_i2c_write( uint8_t i2csel, uint32_t bytecount, uint8_t * wrbyte )
{
	uint8_t retcode = I2C_CMD_EXECUTED;

	SELECT_I2C_CHAN( i2csel );
	while( !READ_BIT(_i2csel->SR1, I2C_SR1_TXE )){}				/// Wait for ADDR tx *
	
	for( uint32_t i = 0; i < bytecount; i++ )
	{
		WRITE_REG(_i2csel->DR, *wrbyte++ );						/// Send wrbyte
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_TXE )){}			/// Wait for ADDR tx *
	}

	while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}				/// Wait for last byte tx *
}

/// @brief Used after i2c_start(), write device ID and read incoming bytes. i2c_stop() is NOT required.
/// @brief Performs "Method2" I2C Read (refer to RM0008 document)
/// @param i2csel : I2C1 or I2C2 for STM32F1xx
/// @param devid : Target slave ID with LSB as read mode
/// @param bytecount : Number of bytes to read (min: 1); must match rdbyte[] size
/// @param rdbyte : Pass-by-reference pointer to byte-array, for storing read bytes
void stm32f1_i2c_read( uint8_t i2csel, uint8_t devid, uint32_t bytecount, uint8_t * rdbyte )
{
	SELECT_I2C_CHAN( i2csel );
	WRITE_REG(_i2csel->DR, devid );								/// Write device id

	if( bytecount == 1U )
	{
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn off Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		SET_BIT(_i2csel->CR1, I2C_CR1_STOP );					/// Generate Stop
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}		/// Wait until byte rx is done *
		*rdbyte = READ_REG(_i2csel->DR );						/// Read Data Register
	}
	else if( bytecount == 2U )
	{
		SET_BIT(_i2csel->CR1, I2C_CR1_POS );					/// Set POS bit
		SET_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn on Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn off Master Ack
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}			/// Wait until Shift Register and Data Register are filled *
		SET_BIT(_i2csel->CR1, I2C_CR1_STOP );					/// Generate Stop
		*rdbyte++ = READ_REG(_i2csel->DR );						/// Read second-to-last byte
		*rdbyte = READ_REG(_i2csel->DR );						/// Read last byte
		CLEAR_BIT(_i2csel->CR1, I2C_CR1_POS );					/// Cleanup: Turn off POS bit
	}
	else
	{
		while( !READ_BIT(_i2csel->SR1, I2C_SR1_ADDR )){}		/// Wait until AddrStatus is asserted
		READ_REG(_i2csel->SR2 );								/// Read SR2 to clear SR1[ADDR] bit
		SET_BIT(_i2csel->CR1, I2C_CR1_ACK );					/// Turn on Master Ack
		
		while( bytecount > 0 )
		{
			if( bytecount > 3U )
			{
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until byte rx is done *
				*rdbyte++ = READ_REG(_i2csel->DR );				/// Read incoming byte
				bytecount--;
			}
			else
			{
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until data[n-2] is received *
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_BTF )){}		/// Wait until data[n-1] is received *
				CLEAR_BIT(_i2csel->CR1, I2C_CR1_ACK );				/// Turn off Master Ack
				*rdbyte++ = READ_REG(_i2csel->DR );					/// Read data[n-2] to continue reading data[n] in I2C
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}	/// Wait until data[n] is received *
				SET_BIT(_i2csel->CR1, I2C_CR1_STOP );				/// Generate Stop
				*rdbyte++ = READ_REG(_i2csel->DR );					/// Read data[n-1] to continue reading data[n] in I2C
				bytecount--;
				while( !READ_BIT(_i2csel->SR1, I2C_SR1_RXNE )){}
				*rdbyte = READ_REG(_i2csel->DR );					/// Read data[n]
			}
		}
	}
}

static void stm32f1_i2c_config_and_enable( I2C_TypeDef * i2csel, uint32_t mcu_freq_mhz, uint32_t mcu_period_ns, I2C_Mode i2cspeed )
{
	/// Toggle I2Cx Reset :
	SET_BIT( i2csel->CR1, I2C_CR1_SWRST );									
	CLEAR_BIT( i2csel->CR1, I2C_CR1_SWRST );				

	uint32_t mode_freq = ( i2cspeed == I2C_SM_100KHZ ? 100000U : 400000U );
	uint32_t mode_period_ns = ( i2cspeed == I2C_SM_100KHZ ? SM_PERIOD_NS : FM_PERIOD_NS );

	uint32_t tre_ns = i2cspeed == I2C_SM_100KHZ ? SM_TRE_NS : FM_TRE_NS; 
	uint32_t ccr_freq = (uint16_t)( ( mode_period_ns / 2U ) / mcu_period_ns );
	uint32_t trise_val = (uint16_t)( ( tre_ns / mcu_period_ns ) + 1U );

	/// Standard mode: i2c_tpos / sys_period = (1/100e3) / 2 ) / (1/8e6) = 40 (0x28)

	CLEAR_BIT( i2csel->CR1, I2C_CR1_PE );							/// Disable I2C
	MODIFY_REG( i2csel->CR2, I2C_CR2_FREQ, mcu_freq_mhz );			/// Set system FREQ = 8MHz
	MODIFY_REG( i2csel->CCR, I2C_CCR_CCR, ccr_freq );				/// Standard mode, set frequency
	MODIFY_REG( i2csel->TRISE, I2C_TRISE_TRISE, trise_val );		/// Set SCL rising edge time
	SET_BIT( i2csel->CR1, I2C_CR1_PE );								/// Enable I2C
}

static uint8_t wait_timeout( I2C_TypeDef * i2csel, uint16_t statusbit, I2C_Error_Code errcode )
{
	uint8_t retval = I2C_CMD_EXECUTED;

	while( !READ_BIT( i2csel->SR1, statusbit )){}
	//for( uint32_t i = 0U; i < ( TIMEOUT_LIMIT * TIMEOUT_MULT ); i++ )
	//{
	//	if( READ_BIT( i2csel->SR1, statusbit ))
	//		break;
	//	else if( i == ( TIMEOUT_LIMIT * TIMEOUT_MULT ) - 1 )
	//		retval = errcode;
	//}

	return retval;
}
