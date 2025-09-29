///===================================================================
/// File Name: bme680_def.h
/// Type     : STM32 C-header
/// Purpose  : BME680 Driver
/// Version  : 1.0
///===================================================================
/// Description
///     * Definition file for Bosch BME860 sensor
///     * Gas, pressure, temperature, humidity sensor
///     * I2C or SPI communication
///===================================================================
/// Table of Contents
///		* Sec_1_0 : Constants
///     * Sec_2_0 : Memory Map
///		* Sec_3_0 : Structures
///		* Sec_4_0 : Bit Definitions
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Aug-29 / G.RUIZ
///		* Initial release
///===================================================================


#ifndef BME680_DEF__
#define BME680_DEF__

#include <stdint.h>



///===================================================================
///							Sec_1_0 CONSTANTS
///===================================================================

#define BME680_I2C_ADDR_GND					( 0x76UL		)		/// I2C address when SDO is strapped to GND
#define BME680_I2C_ADDR_VDDIO				( 0X77UL		)		/// I2C address when SDO is strapped to VDDIO

#define BME680_I2C_SM_FREQ					( 100e3U		)		/// I2C Standard Mode
#define BME680_I2C_FM_FREQ					( 400e3U		)		/// I2C Fast Mode
#define BME680_I2C_HM_FREQ					( 1e6U 			)		/// I2C High Speed Mode
#define BME680_SPI_CLK_FREQ_MAX				( 10e6U			)		/// SPI max frequency
#define BME680_RESET_DURATION_NS			( 10000000U		)		/// Reset duration 10ms

#define BME680_RESET_ENABLE					( 0xB6UL		)		/// Reset command byte
#define BME680_CHIPID_DEFAULT				( 0x61UL		)		/// Chip ID
#define BME680_TEMP_MSB_DEFAULT				( 0x80UL		)		/// Temperature MSB default state
#define BME680_PRESS_MSB_DEFAULT			( 0x80UL		)		/// Pressure MSB default value

#define BME680_VARIANT_LOW					( 0x00UL		)
#define BME680_VARIANT_HIGH					( 0x01UL		)


///===================================================================
///						Sec_2_0 TYPE DEFINITIONS
///===================================================================


/// @brief Sensor Modes
typedef enum
{
    MODE_SLEEP,
    MODE_FORCE
} SensorMode;

/// @brief Humidity Oversampling
typedef enum
{
	OSRS_H_SKIPPED,
	OSRS_H_X1,
	OSRS_H_X2,
	OSRS_H_X4,
	OSRS_H_X8,
	OSRS_H_X16
} OSRS_H;


/// @brief Temperature Oversampling
typedef enum
{
	OSRS_T_SKIPPED,
	OSRS_T_X1,
	OSRS_T_X2,
	OSRS_T_X4,
	OSRS_T_X8,
	OSRS_T_X16
} OSRS_T;


/// @brief Pressure Oversampling
typedef enum
{
	OSRS_P_SKIPPED,
	OSRS_P_X1,
	OSRS_P_X2,
	OSRS_P_X4,
	OSRS_P_X8,
	OSRS_P_X16
} OSRS_P;


/// @brief IIR Filter Control
typedef enum
{
	COEFF_0,
	COEFF_1,
	COEFF_3,
	COEFF_7,
	COEFF_15,
	COEFF_21,
	COEFF_63,
	COEFF_127
} FilterCoeff;


/// @brief Gas Sensor Wait Time
typedef enum
{
	GASWAIT_X1,
	GASWAIT_X4,
	GASWAIT_X16,
	GASWAIT_X64
} GasWaitMult;


/// @brief Heat Profile Selection
typedef enum
{
	NBCONV_0,
	NBCONV_1,
	NBCONV_2,
	NBCONV_3,
	NBCONV_4,
	NBCONV_5,
	NBCONV_6,
	NBCONV_7,
	NBCONV_8,
	NBCONV_9,
} NBConv;


///===================================================================
///						  Sec_3_0 STRUCTURES
///===================================================================






///===================================================================
///						  Sec_4_0 MEMORY MAP
///===================================================================

#define BME680_ADDR_VARIANT_BASE			( 0xF0UL )
#define BME680_ADDR_CAL_SETA_BASE			( 0xE1UL )
#define BME680_ADDR_CAL_SETB_BASE			( 0x8AUL )
#define BME680_ADDR_RESET_BASE				( 0xE0UL )
#define BME680_ADDR_CHIPID_BASE				( 0xD0UL )
#define BME680_ADDR_CONFIG_BASE				( 0x75UL )
#define BME680_ADDR_CTRL_BASE				( 0x70UL )
#define BME680_ADDR_GAS_WAIT_X_BASE			( 0x64UL )
#define BME680_ADDR_RES_HEAT_X_BASE			( 0x5AUL )
#define BME680_ADDR_IDAC_HEAT_X_BASE		( 0x50UL )
#define BME680_ADDR_GAS_BASE				( 0x2AUL )
#define BME680_ADDR_HUM_BASE				( 0x25UL )
#define BME680_ADDR_TEMP_BASE				( 0x22UL )
#define BME680_ADDR_PRESS_BASE				( 0x1FUL )
#define BME680_ADDR_MEAS_STATUS_0_BASE		( 0x1DUL )
#define BME680_ADDR_RES_RANGE_SW_ERR_BASE	( 0x04UL )
#define BME680_ADDR_RES_HEAT_RANGE_BASE		( 0x02UL )
#define BME680_ADDR_RES_HEAT_VAL_BASE		( 0x00UL )

#define BME680_ADDR_CTRL_MEAS_BASE			( BME680_ADDR_CTRL_BASE + 0x04UL )
#define BME680_ADDR_CTRL_HUM_BASE			( BME680_ADDR_CTRL_BASE + 0x02UL )
#define BME680_ADDR_CTRL_GAS1_BASE			( BME680_ADDR_CTRL_BASE + 0x01UL )
#define BME680_ADDR_CTRL_GAS0_BASE			( BME680_ADDR_CTRL_BASE )
#define BME680_ADDR_GAS_WAIT_9_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x09UL )
#define BME680_ADDR_GAS_WAIT_8_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x08UL )
#define BME680_ADDR_GAS_WAIT_7_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x07UL )
#define BME680_ADDR_GAS_WAIT_6_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x06UL )
#define BME680_ADDR_GAS_WAIT_5_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x05UL )
#define BME680_ADDR_GAS_WAIT_4_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x04UL )
#define BME680_ADDR_GAS_WAIT_3_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x03UL )
#define BME680_ADDR_GAS_WAIT_2_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x02UL )
#define BME680_ADDR_GAS_WAIT_1_BASE			( BME680_ADDR_GAS_WAIT_X_BASE + 0x01UL )
#define BME680_ADDR_GAS_WAIT_0_BASE			( BME680_ADDR_GAS_WAIT_X_BASE )
#define BME680_ADDR_RES_HEAT_9_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x09UL )
#define BME680_ADDR_RES_HEAT_8_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x08UL )
#define BME680_ADDR_RES_HEAT_7_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x07UL )
#define BME680_ADDR_RES_HEAT_6_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x06UL )
#define BME680_ADDR_RES_HEAT_5_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x05UL )
#define BME680_ADDR_RES_HEAT_4_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x04UL )
#define BME680_ADDR_RES_HEAT_3_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x03UL )
#define BME680_ADDR_RES_HEAT_2_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x02UL )
#define BME680_ADDR_RES_HEAT_1_BASE			( BME680_ADDR_RES_HEAT_X_BASE + 0x01UL )
#define BME680_ADDR_RES_HEAT_0_BASE			( BME680_ADDR_RES_HEAT_X_BASE )
#define BME680_ADDR_GAS_R_LSB_BASE			( BME680_ADDR_GAS_BASE + 0x01UL )
#define BME680_ADDR_GAS_R_MSB_BASE			( BME680_ADDR_GAS_BASE )
#define BME680_ADDR_HUM_LSB_BASE			( BME680_ADDR_HUM_BASE + 0x01UL )
#define BME680_ADDR_HUM_MSB_BASE			( BME680_ADDR_HUM_BASE )
#define BME680_ADDR_TEMP_XLSB_BASE			( BME680_ADDR_TEMP_BASE + 0x02UL )
#define BME680_ADDR_TEMP_LSB_BASE			( BME680_ADDR_TEMP_BASE + 0x01UL )
#define BME680_ADDR_TEMP_MSB_BASE			( BME680_ADDR_TEMP_BASE )
#define BME680_ADDR_PRESS_XLSB_BASE			( BME680_ADDR_PRESS_BASE + 0x02UL )
#define BME680_ADDR_PRESS_LSB_BASE			( BME680_ADDR_PRESS_BASE + 0x01UL )
#define BME680_ADDR_PRESS_MSB_BASE			( BME680_ADDR_PRESS_BASE )



///===================================================================
///						Sec_3_0 BIT DEFINITIONS
///===================================================================


///************************ RESET Register ***************************/
#define RESET_MASK							( 0xFFUL )


///*********************** CHIPID Register ***************************/
#define CHIPID_MASK							( 0xFFUL )


///*********************** CONFIG Register ***************************/
#define CONFIG_SPI_3W_EN_Pos				( 0U )
#define CONFIG_SPI_3W_EN_Mask				( 0x01UL << CONFIG_SPI_3W_EN_Pos )
#define CONFIG_SPI_3W_EN					( CONFIG_SPI_3W_EN_Mask )

#define CONFIG_FILTER_Pos					( 2U )
#define CONFIG_FILTER_Mask					( 0x07UL << CONFIG_FILTER_Pos )
#define CONFIG_FILTER						( CONFIG_FILTER_Mask )

///********************* CTRL_MEAS Register **************************/
#define CTRL_MEAS_MODE_Pos					( 0U )
#define CTRL_MEAS_MODE_Mask					( 0x03UL << CTRL_MEAS_MODE_Pos )
#define CTRL_MEAS_MODE						( CTRL_MEAS_MODE_Mask )

#define CTRL_MEAS_OSRS_P_Pos				( 2U )
#define CTRL_MEAS_OSRS_P_Mask				( 0x07UL << CTRL_MEAS_OSRS_P_Pos )
#define CTRL_MEAS_OSRS_P					( CTRL_MEAS_OSRS_P_Mask )

#define CTRL_MEAS_OSRS_T_Pos				( 5U )
#define CTRL_MEAS_OSRS_T_Mask				( 0x07UL << CTRL_MEAS_OSRS_T_Pos )
#define CTRL_MEAS_OSRS_T					( CTRL_MEAS_OSRS_T_Mask )


///********************** CTRL_HUM Register **************************/
#define CTRL_HUM_OSRS_H_Pos					( 0U )
#define CTRL_HUM_OSRS_H_Mask				( 0x07UL << CTRL_HUM_OSRS_H_Pos )
#define CTRL_HUM_OSRS_H						( CTRL_HUM_OSRS_H_Mask )

#define CTRL_HUM_SPI_3W_INT_EN_Pos			( 6U )
#define CTRL_HUM_SPI_3W_INT_EN_Mask			( 0x01UL << CTRL_HUM_SPI_3W_INT_EN_Pos )
#define CTRL_HUM_SPI_3W_INT_EN				( CTRL_HUM_SPI_3W_INT_EN_Mask )


///********************* CTRL_GAS1 Register **************************/
#define CTRL_GAS_1_NB_CONV_Pos				( 0U )
#define CTRL_GAS_1_NB_CONV_Mask				( 0x0FUL << CTRL_GAS_1_NB_CONV_Pos )
#define CTRL_GAS_1_NB_CONV					( CTRL_GAS_1_NB_CONV_Mask )

#define CTRL_GAS_1_RUN_GAS_Pos				( 4U )
#define CTRL_GAS_1_RUN_GAS_Mask				( 0x01UL << CTRL_GAS_1_RUN_GAS_Pos )
#define CTRL_GAS_1_RUN_GAS					( CTRL_GAS_1_RUN_GAS_Mask )


///********************** CTRL_GAS0 Register *************************/
#define CTRL_GAS_0_HEAT_OFF_Pos				( 3U )
#define CTRL_GAS_0_HEAT_OFF_Mask			( 0x01UL << CTRL_GAS_0_HEAT_OFF_Pos )
#define CTRL_GAS_0_HEAT_OFF					( CTRL_GAS_0_HEAT_OFF_Mask )


///********************* GAS_WAIT_X Registers ************************/
#define GAS_WAIT_X_Pos						( 0U )
#define GAS_WAIT_X_Mask						( 0xFFUL << GAS_WAIT_X_Pos )
#define GAS_WAIT_X							( GAS_WAIT_X_Mask )


///********************* RES_HEAT_X Register *************************/
#define RES_HEAT_X_Pos						( 0U )
#define RES_HEAT_X_Mask						( 0xFFUL << RES_HEAT_X_Pos )
#define RES_HEAT_X							( RES_HEAT_X_Mask )


///********************* IDAC_HEAT_X Register ************************/
#define IDAC_HEAT_X_Pos						( 0U )
#define IDAC_HEAT_X_Mask					( 0xFFUL << IDAC_HEAT_X_Pos )
#define IDAC_HEAT_X							( IDAC_HEAT_X_Mask )


///********************** GAS_R_LSB Register *************************/
#define GAS_R_LSB_GAS_RANGE_R_Pos			( 0U )
#define GAS_R_LSB_GAS_RANGE_R_Mask			( 0x0FUL << GAS_R_LSB_GAS_RANGE_R_Pos )
#define GAS_R_LSB_GAS_RANGE_R				( GAS_R_LSB_GAS_RANGE_R_Mask )

#define GAS_R_LSB_HEAT_STAB_R_Pos			( 4U )
#define GAS_R_LSB_HEAT_STAB_R_Mask			( 0x01UL << GAS_R_LSB_HEAT_STAB_R_Pos )
#define GAS_R_LSB_HEAT_STAB_R				( GAS_R_LSB_HEAT_STAB_R_Mask )

#define GAS_R_LSB_GAS_VALID_R_Pos			( 5U )
#define GAS_R_LSB_GAS_VALID_R_Mask			( 0x01UL << GAS_R_LSB_GAS_VALID_R_Pos )
#define GAS_R_LSB_GAS_VALID_R				( GAS_R_LSB_GAS_VALID_R_Mask )

#define GAS_R_LSB_GAS_R_Pos					( 6U )
#define GAS_R_LSB_GAS_R_Mask				( 0x03UL << GAS_R_LSB_GAS_R_Pos )
#define GAS_R_LSB_GAS_R						( GAS_R_LSB_GAS_R_Mask )


///********************** GAS_R_MSB Register *************************/
#define GAS_R_MSB_GAS_R_Pos					( 0U )
#define GAS_R_MSB_GAS_R_Mask				( 0xFFUL << GAS_R_MSB_GAS_R_Pos )
#define GAS_R_MSB_GAS_R						( GAS_R_MSB_GAS_R_Mask )


///*********************** HUM_LSB Register **************************/
#define HUM_LSB_Pos							( 0U )
#define HUM_LSB_Mask						( 0xFFUL << HUM_LSB_Pos )
#define HUM_LSB								( HUM_LSB_Mask )


///*********************** HUM_MSB Register **************************/
#define HUM_MSB_Pos							( 0U )
#define HUM_MSB_Mask						( 0xFFUL << HUM_MSB_Pos )
#define HUM_MSB								( HUM_MSB_Mask )


///********************** TEMP_XLSB Register *************************/
#define TEMP_XLSB_Pos						( 4U )
#define TEMP_XLSB_Mask						( 0xF0UL << TEMP_XLSB_Pos )
#define TEMP_XLSB							( TEMP_XLSB_Mask )


///********************** TEMP_LSB Register **************************/
#define TEMP_LSB_Pos						( 0U )
#define TEMP_LSB_Mask						( 0xFFUL << TEMP_LSB_Pos )
#define TEMP_LSB							( TEMP_LSB_Mask )


///********************** TEMP_MSB Register **************************/
#define TEMP_MSB_Pos						( 0U )
#define TEMP_MSB_Mask						( 0xFFUL << TEMP_MSB_Pos )
#define TEMP_MSB							( TEMP_MSB_Mask )


///********************* PRESS_XLSB Register *************************/
#define PRESS_XLSB_Pos						( 4U )
#define PRESS_XLSB_Mask						( 0xF0UL << PRESS_XLSB_Pos )
#define PRESS_XLSB							( PRESS_XLSB_Mask )


///********************** PRESS_LSB Register *************************/
#define PRESS_LSB_Pos						( 0U )
#define PRESS_LSB_Mask						( 0xFFUL << PRESS_LSB_Pos )
#define PRESS_LSB							( PRESS_LSB_Mask )


///********************** PRESS_MSB Register *************************/
#define PRESS_MSB_Pos						( 0U )
#define PRESS_MSB_Mask						( 0xFFUL << PRESS_MSB_Pos )
#define PRESS_MSB							( PRESS_MSB_Mask )


///******************** EAS_STATUS_0 Register ************************/
#define MEAS_STATUS_0_GAS_MEAS_INDEX_0_Pos	( 0U )
#define MEAS_STATUS_0_GAS_MEAS_INDEX_0_Mask	( 0x0FUL << MEAS_STATUS_0_GAS_MEAS_INDEX_0_Pos )
#define MEAS_STATUS_0_GAS_MEAS_INDEX_0		( MEAS_STATUS_0_GAS_MEAS_INDEX_0_Mask )

#define MEAS_STATUS_0_MEASURING_Pos			( 5U )
#define MEAS_STATUS_0_MEASURING_Mask		( 0x01UL << MEAS_STATUS_0_MEASURING_Pos )
#define MEAS_STATUS_0_MEASURING				( MEAS_STATUS_0_MEASURING_Mask )

#define MEAS_STATUS_0_GAS_MEASURING_Pos		( 6U )
#define MEAS_STATUS_0_GAS_MEASURING_Mask	( 0x01UL << MEAS_STATUS_0_GAS_MEASURING_Pos )
#define MEAS_STATUS_0_GAS_MEASURING			( MEAS_STATUS_0_GAS_MEASURING_Mask )

#define MEAS_STATUS_0_NEW_DATA_0_Pos		( 7U )
#define MEAS_STATUS_0_NEW_DATA_0_Mask		( 0x01UL << MEAS_STATUS_0_NEW_DATA_0_Pos )
#define MEAS_STATUS_0_NEW_DATA_0			( MEAS_STATUS_0_NEW_DATA_0_Mask )


///******************* RES_HEAT_RANGE Register ***********************/
#define RES_HEAT_RANGE_Mask					( 0x30UL )


///******************** RANGE_SW_ERR Register ************************/
#define RANGE_SW_ERR_Mask					( 0xF0UL )


///*********************** PAR_H1 Register ***************************/
#define PAR_H1_LSB_Mask						( 0xF0UL )



#endif ///BME680_DEF__