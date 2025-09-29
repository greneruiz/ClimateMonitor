///===================================================================
/// File Name: BME680Ctrl.h
/// Type     : STM32 C-header
/// Purpose  : Driver for BME680 Temp/Pres/Hum/Gas sensor
/// Version  : 1.0
///===================================================================
/// Description
///		* For Bosch BME680
///		* I2C communication
///		* Integer-based (STM32F1); disabled floating-point equations
///		* Disabled Gas Sensor functionality - not enough SRAM
///
///	Resources Used:
///		* TIM2 - wait delays
///		* I2C1 - PB6 SCL; PB7 SDA; or...
///		* I2C2 - PB10 SCL; PB11 SDA
///
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Sep-29 / G.RUIZ
///		* Initial release
///===================================================================


#ifndef BME680CTRL_H_
#define BME680CTRL_H_

//#include <stdlib.h>
//#include <stdint.h>
//#include <stddef.h>

#include "stm32f1_waitdly.h"


/// @attention Change according to current MCU clock settings
#define MCU_FREQUENCY_MHZ	( 8U		)
#define MCU_PERIOD_NS		( 125U		)
#define MCU_FREQUENCY_HZ	( 8000000U	)

/// @attention Undefine to enable floating-point results
#define BME680_RETURNS_FLOAT
#undef BME680_RETURNS_FLOAT

/// @attention We disable gas measurements because STM32F1 has not enough SRAM and FPU for this...
/// if enabling this, add "-lgcc" to linker @ Makefile, or enable FPU...
#define ENABLE_GAS_MEAS
#undef ENABLE_GAS_MEAS

#define RETRY_COUNT			( 5U		)
#define BOOTUP_FREQ_HZ		( 100U		)
#define MEASDLY_FREQ_HZ		( 50U		)

/// @attention Settings for delays
#define RST_DLY_PRESCALER	( 9999U		)
#define RST_DLY_CNT_10MS	( 8U		)
//#define RST_DLY_CNT_10MS	( MCU_FREQUENCY_HZ / (( RST_DLY_PRESCALER + 1U ) * BOOTUP_FREQ_HZ ))

#define MEASDLY_PRESCALER	( 9999U		)
#define MEASDLY_CNT_100MS	( 79U		)
//#define MEASDLY_CNT_20MS	( MCU_FREQUENCY_HZ / (( RST_DLY_PRESCALER + 1U ) * MEASDLY_FREQ_HZ ))


/// @brief Error Codes
typedef enum
{
	BME680_ERR,
	BME680_OK,
} BME680_Err_Code;


/// @brief CalData structure; filled by reading the device
struct BME680_Caldata
{
	uint16_t par_t1;
	int16_t par_t2;
	int8_t par_t3;

	uint16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int16_t par_p4;
	int16_t par_p5;
	int8_t par_p6;
	int8_t par_p7;
	int16_t par_p8;
	int16_t par_p9;
	uint8_t par_p10;

	uint16_t par_h1;
	uint16_t par_h2;
	int8_t par_h3;
	int8_t par_h4;
	int8_t par_h5;
	uint8_t par_h6;
	int8_t par_h7;

	int8_t par_gh1;
	int16_t par_gh2;
	int8_t par_gh3;

	int8_t range_sw_err;
	uint8_t res_heat_range;
	int8_t res_heat_val;

	/// Variable to store the intermediate temperature coefficient:
#ifndef BME680_RETURNS_FLOAT
    int32_t t_fine;		
#else
    float t_fine;
#endif
};


/// @brief ADC data structure
struct BME680_ADCData
{
	/// @brief Data info section:
	uint8_t status;
	uint8_t meas_index;

	/// @brief Raw ADC values:
	uint32_t temp_adc;
	uint32_t pres_adc;
	uint16_t hum_adc;
	uint8_t gas_range;
	uint16_t gas_res_adc;

	/// @brief Calculated values:
#ifndef BME680_RETURNS_FLOAT
	int16_t temperature;		/// Degrees Celsius x100
	uint32_t pressure;			/// Pascals
	uint32_t humidity;			/// % Relative Humidity x1000
	uint32_t gas_resistance;	/// Ohms
#else
	float temperature;			/// Degrees Celsius
	float pressure;				/// Pascals
	float humidity;				/// % Relative Humidity x1000
	float gas_resistance;		/// Ohms
#endif
};


/// @brief Sensor instance structure
struct BME680_Instance
{
	/// @brief Device info section:
	uint8_t i2c_channel;		/// 0 = none; 1 = I2C1; 2 = I2C2
	uint8_t chip_id;
	uint8_t variant_id;			/// 0 = GAS_LOW; 1 = GAS_HIGH

	/// @brief Device config section:
	uint8_t run_gas;			/// 1 = Enable gas measurements
	uint8_t hum_osrs;			/// 0 to 5
	uint8_t temp_osrs;			/// 0 to 5
	uint8_t pres_osrs;			/// 0 to 5
	uint8_t filter;				/// 0 to 7
	uint8_t nb_conv;			/// Max value: 0x09
	uint8_t gas_wait[10];		/// Max array depth: 10; 
	uint8_t res_heat[10];		/// Max array depth: 10; in degrees C

	int8_t amb_temp;
	struct BME680_Caldata caldata;
};


/// The user must first malloc a BME680_Instance, giving it an i2c_channel.
/// 	The device config section is then filled; for gas_wait and res_heat, the user
/// 	must malloc the device's respective channels (max of 10).
/// The chip_id, variant_id, and caldata struct is filled by calling BME680_Initialize().
/// The user must also malloc the BME680_ADCData for each BME680_Instance.
///		For each BME680_Instance, it is possible to have multiple BME680_ADCData's,
///		if the user is planning to multi-read the results from the device.



extern uint8_t BME680_Initialize(struct BME680_Instance * mysensor);
extern uint8_t BME680_Setup_and_Run(struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata, uint8_t measonly);
extern uint8_t BME680_GetResults(struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata);

extern uint8_t BME680_Set_Oversampling_and_Filter(struct BME680_Instance * mysensor);

#ifdef ENABLE_GAS_MEAS
	extern uint8_t BME680_Set_GasHeater(struct BME680_Instance * mysensor);
#endif

extern void BME680_RunMeas(struct BME680_Instance * mysensor);

extern void BME680_Sample_Sensor_Setup(struct BME680_Instance * mysensor);



#endif ///BME680CTRL_H_