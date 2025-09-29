///===================================================================
/// File Name: BME680Ctrl.c
/// Type     : STM32 C-source
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


/// @attention To-do: replace items marked ///[BLOCKING] with non-blocking methods

#include <stdio.h>

#include "bme680_def.h"
#include "bme680_i2c.h"
#include "BME680Ctrl.h"


#define CONCAT_BYTES( msb, lsb )	(( (uint16_t)msb << 8 ) | (uint16_t)lsb )



uint8_t BME680_Initialize( struct BME680_Instance * mysensor );
uint8_t BME680_Setup_and_Run( struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata, uint8_t measonly );
uint8_t BME680_Set_Oversampling_and_Filter( struct BME680_Instance * mysensor );

#ifdef ENABLE_GAS_MEAS
	uint8_t BME680_Set_GasHeater( struct BME680_Instance * mysensor );
	static uint8_t calc_gas_wait( uint16_t duration );
#endif /*ENABLE_GAS_MEAS*/

void BME680_RunMeas( struct BME680_Instance * mysensor );
static void BME680_Get_CalData( struct BME680_Instance * mysensor );
void BME680_Sample_Sensor_Setup( struct BME680_Instance * mysensor );

#ifndef BME680_RETURNS_FLOAT
	static int16_t calc_temperature( struct BME680_Instance * mysensor, uint32_t temp_adc );
	static uint32_t calc_pressure( struct BME680_Instance * mysensor, uint32_t pres_adc );
	static uint32_t calc_humidity( struct BME680_Instance * mysensor, uint16_t hum_adc );

	#ifdef ENABLE_GAS_MEAS
		static uint32_t calc_gas_resistance_low( struct BME680_Instance * mysensor, uint16_t gas_res_adc, uint8_t gas_range );
		static uint32_t calc_gas_resistance_high( uint16_t gas_res_adc, uint8_t gas_range );
		static uint8_t calc_res_heat( struct BME680_Instance * mysensor, uint16_t temp );
	#endif /*ENABLE_GAS_MEAS*/
#else
	static float calc_temperature( struct BME680_Instance * mysensor, uint32_t temp_adc );
	static float calc_pressure( struct BME680_Instance * mysensor, uint32_t pres_adc );
	static float calc_humidity( struct BME680_Instance * mysensor, uint16_t hum_adc );

	#ifdef ENABLE_GAS_MEAS
		static float calc_gas_resistance_low( struct BME680_Instance * mysensor, uint16_t gas_res_adc, uint8_t gas_range );
		static float calc_gas_resistance_high( uint16_t gas_res_adc, uint8_t gas_range );
		static uint8_t calc_res_heat( struct BME680_Instance * mysensor, uint16_t temp );
	#endif /*ENABLE_GAS_MEAS*/
#endif

uint8_t BME680_GetResults( struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata );
static void BME680_Get_CalData( struct BME680_Instance * mysensor );



/// @return 0x01UL = Chip ID matched
uint8_t BME680_Initialize( struct BME680_Instance * mysensor )
{
	uint8_t _retcode = BME680_OK;
	uint8_t _wrbyte[2] = { BME680_ADDR_RESET_BASE, BME680_RESET_ENABLE };

	/// Initialize MCU I2C, then perform BME680 device soft-reset
	bme680_i2c_initialize( MCU_FREQUENCY_MHZ, MCU_PERIOD_NS, mysensor->i2c_channel );
	bme680_i2c_write( mysensor->i2c_channel, 2U, _wrbyte );	

	/// Wait for 100ms
	init_tim2( MEASDLY_PRESCALER, MEASDLY_CNT_100MS );
	tim2_wait_delay();

	/// Read Chip ID and Variant ID
	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_CHIPID_BASE, 1U, &mysensor->chip_id );
	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_VARIANT_BASE, 1U, &mysensor->variant_id );
//	_retcode = mysensor->chip_id == BME680_CHIPID_DEFAULT ? BME680_OK : BME680_ERR;

/*	/// Let's perform a scratchpad test at addr: GAS_WAIT_0 ...
	uint8_t _rdbyte = 0x00UL;
	_wrbyte[0] = BME680_ADDR_GAS_WAIT_0_BASE;
	_wrbyte[1] = 0xA5UL;
	bme680_i2c_write( mysensor->i2c_channel, 2U, _wrbyte );
	bme680_i2c_read( mysensor->i2c_channel, _wrbyte[0], 1U, &_rdbyte );
	_retcode &= _rdbyte == _wrbyte[1] ? BME680_OK : BME680_ERR;
*/
	/// Wait for 100ms: If there's no delay here, the measurement calculations somehow go wrong...
	init_tim2( MEASDLY_PRESCALER, MEASDLY_CNT_100MS );
	tim2_wait_delay();

//	if(_retcode == BME680_OK )
		BME680_Get_CalData( mysensor );

	return _retcode;
}

uint8_t BME680_Setup_and_Run( struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata, uint8_t measonly )
{
	if( measonly == 0x00 )
	{
		BME680_Set_Oversampling_and_Filter( mysensor );
	#ifdef ENABLE_GAS_MEAS
		if( mysensor->run_gas ) BME680_Set_GasHeater( mysensor );
	#endif
	}

	BME680_RunMeas( mysensor );

	/// Wait for 20ms
	init_tim2( MEASDLY_PRESCALER, MEASDLY_CNT_100MS - 1U );
	tim2_wait_delay();

	return BME680_GetResults( mysensor, adcdata );
}

uint8_t BME680_Set_Oversampling_and_Filter( struct BME680_Instance * mysensor )
{
	uint8_t _wrbyte[6];

	_wrbyte[0] = BME680_ADDR_CTRL_HUM_BASE,
	_wrbyte[1] = mysensor->hum_osrs & CTRL_HUM_OSRS_H,
	_wrbyte[2] = BME680_ADDR_CTRL_MEAS_BASE,

	_wrbyte[3] = ( mysensor->temp_osrs << CTRL_MEAS_OSRS_T_Pos ) & CTRL_MEAS_OSRS_T;
	_wrbyte[3] |= ( mysensor->pres_osrs << CTRL_MEAS_OSRS_P_Pos ) & CTRL_MEAS_OSRS_P;
	_wrbyte[3] &= ~CTRL_MEAS_MODE_Mask;

	_wrbyte[4] = BME680_ADDR_CONFIG_BASE;
	_wrbyte[5] = ( mysensor->filter << CONFIG_FILTER_Pos ) & CONFIG_FILTER;

	bme680_i2c_write( mysensor->i2c_channel, sizeof(_wrbyte ), _wrbyte );

	return 1U;
}

#ifdef ENABLE_GAS_MEAS
	uint8_t BME680_Set_GasHeater( struct BME680_Instance * mysensor )
	{
		uint8_t _wrbyte[4];
		
		_wrbyte[0] = BME680_ADDR_GAS_WAIT_X_BASE + mysensor->nb_conv;
		_wrbyte[1] = calc_gas_wait( mysensor->gas_wait[mysensor->nb_conv] );
		_wrbyte[2] = BME680_ADDR_RES_HEAT_X_BASE + mysensor->nb_conv;
		_wrbyte[3] = calc_res_heat( mysensor, mysensor->res_heat[mysensor->nb_conv] );
		bme680_i2c_write( mysensor->i2c_channel, sizeof(_wrbyte), _wrbyte );

		return 1U;
	}
#endif

void BME680_RunMeas( struct BME680_Instance * mysensor )
{
	uint8_t _rdbyte;
	uint8_t _wrbyte[4];

	_wrbyte[0] = BME680_ADDR_CTRL_GAS1_BASE;
	_wrbyte[1] = ( mysensor->nb_conv & CTRL_GAS_1_NB_CONV ) | ( mysensor->run_gas & CTRL_GAS_1_RUN_GAS );

	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_CTRL_MEAS_BASE, 1U, &_rdbyte );						/// Read CTRL_MEAS to modify
	_wrbyte[2] = BME680_ADDR_CTRL_MEAS_BASE;
	_wrbyte[3] = ((_rdbyte & ~CTRL_MEAS_MODE_Mask) | MODE_FORCE );

	bme680_i2c_write( mysensor->i2c_channel, sizeof(_wrbyte ), _wrbyte );
}

static void BME680_Get_CalData( struct BME680_Instance * mysensor )
{
	uint8_t _rdbyte[23];

	/// Read 0x00-0x004 (5) :
	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_RES_HEAT_VAL_BASE, 5U, _rdbyte );
	
	mysensor->caldata.res_heat_val = (int8_t)_rdbyte[0];										/// 0x00
	mysensor->caldata.res_heat_range = (_rdbyte[2] & RES_HEAT_RANGE_Mask ) / 16;				/// 0x02
	mysensor->caldata.range_sw_err = ((int8_t)(_rdbyte[4] & RANGE_SW_ERR_Mask )) / 16;			/// 0x04

	/// Read 0xE1-0xEE (14) :
	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_CAL_SETA_BASE, 14U, _rdbyte );

	mysensor->caldata.par_h1 = (uint16_t)(( (uint16_t)_rdbyte[2] << 4U ) | (_rdbyte[1] & PAR_H1_LSB_Mask ));	/// MSB = 0xE3 << 4; LSB = 0xE2[3..0]
	mysensor->caldata.par_h2 = (uint16_t)(( (uint16_t)_rdbyte[0] << 4U ) | (_rdbyte[1] >> 4U ));				/// MSB = 0xE1 << 4; LSB = 0xE2[7..4] <--- Formula does not match Table 13 from datasheet (rev1.9)! Fucking cringe.
	mysensor->caldata.par_h3 = (int8_t)_rdbyte[3];																/// 0xE4
	mysensor->caldata.par_h4 = (int8_t)_rdbyte[4];																/// 0xE5
	mysensor->caldata.par_h5 = (int8_t)_rdbyte[5];																/// 0xE6	
	mysensor->caldata.par_h6 = (uint8_t)_rdbyte[6];																/// 0xE7
	mysensor->caldata.par_h7 = (int8_t)_rdbyte[7];																/// 0xE8
	mysensor->caldata.par_t1 = (uint16_t)( CONCAT_BYTES(_rdbyte[9], _rdbyte[8] ));								/// MSB=0xEA LSB=0xE9
	mysensor->caldata.par_gh2 = (int16_t)( ((int16_t)_rdbyte[11] << 8 ) | (int16_t)_rdbyte[10] );				/// MSB = 0xEC; LSB = 0xEB
	mysensor->caldata.par_gh1 = (int8_t)_rdbyte[12];															/// 0xED
	mysensor->caldata.par_gh3 = (int8_t)_rdbyte[13];															/// 0xEE

	/// Read 0x8A-0xA0 (19) :
	bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_CAL_SETB_BASE, 23U, _rdbyte );

	mysensor->caldata.par_t2 = (int16_t)( CONCAT_BYTES(_rdbyte[1], _rdbyte[0] ));
	mysensor->caldata.par_t3 = (int8_t)_rdbyte[2];
	mysensor->caldata.par_p1 = (uint16_t)( CONCAT_BYTES(_rdbyte[5], _rdbyte[4] ));
	mysensor->caldata.par_p2 = (int16_t)( CONCAT_BYTES(_rdbyte[7], _rdbyte[6] ));
	mysensor->caldata.par_p3 = (int8_t)_rdbyte[8];
	mysensor->caldata.par_p4 = (int16_t)( CONCAT_BYTES(_rdbyte[11], _rdbyte[10] ));
	mysensor->caldata.par_p5 = (int16_t)( CONCAT_BYTES(_rdbyte[13], _rdbyte[12] ));
	mysensor->caldata.par_p6 = (int8_t)_rdbyte[15];
	mysensor->caldata.par_p7 = (int8_t)_rdbyte[14];
	mysensor->caldata.par_p8 = (int16_t)( CONCAT_BYTES(_rdbyte[19], _rdbyte[18] ));
	mysensor->caldata.par_p9 = (int16_t)( CONCAT_BYTES(_rdbyte[21], _rdbyte[20] ));
	mysensor->caldata.par_p10 = _rdbyte[22];
}

/// @brief Sets up BME680_Instance, with one enabled nb_conv channel.
void BME680_Sample_Sensor_Setup( struct BME680_Instance * mysensor )
{
	mysensor->i2c_channel = 0x01UL;
	mysensor->run_gas = 0U;		/// 20250913 TODO: Temporarily disabled. Can't make the integer division to work
	mysensor->hum_osrs = 1U;
	mysensor->temp_osrs = 2U;
	mysensor->pres_osrs = 15U;
	mysensor->filter = 0U;
	mysensor->nb_conv = 0U;
	mysensor->gas_wait[0] = (uint8_t)1000;
	mysensor->res_heat[0] = (uint8_t)350;
	mysensor->amb_temp = 30;	/// hardcoded to 25C
}


///===================================================================
///		Calculations (from Bosch BME680 Library + Datasheet)
///===================================================================

#ifdef ENABLE_GAS_MEAS
	static uint8_t calc_gas_wait( uint16_t duration )
	{
		uint8_t factor = 0;
		uint8_t durval;

		if ( duration >= 0x0FC0 )	/// Cap duration
		{
			durval = 0xFF;
		}
		else
		{
			while( duration > 0x3F )
			{
				duration = duration / 4;
				factor += 1;
			}

			durval = (uint8_t)( duration + ( factor * 64 ));
		}

		return durval;
	}
#endif

#ifndef BME680_RETURNS_FLOAT
	static int16_t calc_temperature( struct BME680_Instance * mysensor, uint32_t temp_adc )
	{
		int64_t var1 = ( (int32_t)temp_adc >> 3 ) - ( (int32_t)mysensor->caldata.par_t1 << 1 );
		int64_t var2 = ( var1 * (int32_t)mysensor->caldata.par_t2 ) >> 11;
		int64_t var3;
		int16_t calc_temp;
	//	int32_t t_fine;
		
		var3 = (( var1 >> 1 ) * ( var1 >> 1 )) >> 12;
		var3 = ( (var3) * ( (int32_t)mysensor->caldata.par_t3 << 4 )) >> 14;
		mysensor->caldata.t_fine = (int32_t)( var2 + var3 );
		calc_temp = (int16_t)((( mysensor->caldata.t_fine * 5 ) + 128 ) >> 8 );	
		
		return calc_temp;
	}

	static uint32_t calc_pressure( struct BME680_Instance * mysensor, uint32_t pres_adc )
	{
		int32_t var1;
		int32_t var2;
		int32_t var3;
		int32_t pressure_comp;	

		/* This value is used to check precedence to multiplication or division
		* in the pressure compensation equation to achieve least loss of precision and
		* avoiding overflows.
		* i.e Comparing value, pres_ovf_check = (1 << 31) >> 1
		*/

		const int32_t pres_ovf_check = INT32_C( 0x40000000 );	

		var1 = (( (int32_t)mysensor->caldata.t_fine ) >> 1 ) - 64000;
		var2 = (((( var1 >> 2 ) * ( var1 >> 2 )) >> 11 ) * (int32_t)mysensor->caldata.par_p6 ) >> 2;
		var2 = var2 + (( var1 * (int32_t)mysensor->caldata.par_p5 ) << 1 );
		var2 = ( var2 >> 2 ) + ( (int32_t)mysensor->caldata.par_p4 << 16 );
		var1 = (((( (var1 >> 2) * ( var1 >> 2 )) >> 13 ) * ( (int32_t)mysensor->caldata.par_p3 << 5 )) >> 3 ) + (( (int32_t)mysensor->caldata.par_p2 * var1 ) >> 1 );
		var1 = var1 >> 18;
		var1 = (( 32768 + var1 ) * (int32_t)mysensor->caldata.par_p1 ) >> 15;

		pressure_comp = 1048576 - pres_adc;
		pressure_comp = (int32_t)(( pressure_comp - (var2 >> 12 )) * ( (uint32_t)3125) );

		if( pressure_comp >= pres_ovf_check )
			pressure_comp = ( (pressure_comp / var1) << 1 );
		else
			pressure_comp = ( (pressure_comp << 1) / var1 );

		var1 = ( (int32_t)mysensor->caldata.par_p9 * (int32_t)((( pressure_comp >> 3 ) * ( pressure_comp >> 3 )) >> 13 )) >> 12;
		var2 = ( (int32_t)( pressure_comp >> 2 ) * (int32_t)mysensor->caldata.par_p8 ) >> 13;
		var3 = ( (int32_t)( pressure_comp >> 8 ) * (int32_t)( pressure_comp >> 8 ) * (int32_t)( pressure_comp >> 8 ) * (int32_t)mysensor->caldata.par_p10 ) >> 17;
		pressure_comp = (int32_t)( pressure_comp ) + (( var1 + var2 + var3 + ( (int32_t)mysensor->caldata.par_p7 << 7 )) >> 4 );	

		return (uint32_t)pressure_comp;
	}

	static uint32_t calc_humidity( struct BME680_Instance * mysensor, uint16_t hum_adc )
	{
		int32_t var1;
		int32_t var2;
		int32_t var3;
		int32_t var4;
		int32_t var5;
		int32_t var6;
		int32_t temp_scaled;
		int32_t calc_hum;	

		temp_scaled = (( (int32_t)mysensor->caldata.t_fine * 5 ) + 128 ) >> 8;
		var1 = (int32_t)( hum_adc - ( (int32_t)( (int32_t)mysensor->caldata.par_h1 * 16 ))) -
				((( temp_scaled * (int32_t)mysensor->caldata.par_h3 ) / ( (int32_t)100 )) >> 1 );
		var2 = ( (int32_t)mysensor->caldata.par_h2 * ((( temp_scaled * (int32_t)mysensor->caldata.par_h4 ) / ( (int32_t)100) ) +
				((( temp_scaled * (( temp_scaled * (int32_t)mysensor->caldata.par_h5 ) / ( (int32_t)100 ))) >> 6 ) / ( (int32_t)100 )) +
				(int32_t)( 1 << 14 ))) >> 10;
		var3 = var1 * var2;
		var4 = (int32_t)mysensor->caldata.par_h6 << 7;
		var4 = (( var4 ) + (( temp_scaled * (int32_t)mysensor->caldata.par_h7 ) / ( (int32_t)100 ))) >> 4;
		var5 = (( var3 >> 14 ) * ( var3 >> 14 )) >> 10;
		var6 = ( var4 * var5 ) >> 1;

		calc_hum = ((( var3 + var6 ) >> 10 ) * ( (int32_t)1000 )) >> 12;
		if ( calc_hum > 100000 ) /* Cap at 100%rH */
			calc_hum = 100000;
		else if (calc_hum < 0)
			calc_hum = 0;

		return (uint32_t)calc_hum;
	}

	#ifdef ENABLE_GAS_MEAS
		static uint32_t calc_gas_resistance_low( struct BME680_Instance * mysensor, uint16_t gas_res_adc, uint8_t gas_range )
		{
			int64_t var1;
			uint64_t var2;
			int64_t var3;
			uint32_t calc_gas_res;
			uint32_t lookup_table1[16] =
			{
				UINT32_C( 2147483647 ), UINT32_C( 2147483647 ), UINT32_C( 2147483647 ), UINT32_C( 2147483647 ), UINT32_C( 2147483647 ),
				UINT32_C( 2126008810 ), UINT32_C( 2147483647 ), UINT32_C( 2130303777 ), UINT32_C( 2147483647 ), UINT32_C( 2147483647 ),
				UINT32_C( 2143188679 ), UINT32_C( 2136746228 ), UINT32_C( 2147483647 ), UINT32_C( 2126008810 ), UINT32_C( 2147483647 ),
				UINT32_C( 2147483647 )
			};

			uint32_t lookup_table2[16] =
			{
				UINT32_C( 4096000000 ), UINT32_C( 2048000000 ), UINT32_C( 1024000000 ), UINT32_C( 512000000 ), UINT32_C( 255744255 ),
				UINT32_C( 127110228 ), UINT32_C( 64000000 ), UINT32_C( 32258064 ), UINT32_C( 16016016 ), UINT32_C( 8000000 ),
				UINT32_C( 4000000 ), UINT32_C( 2000000 ), UINT32_C( 1000000 ), UINT32_C( 500000 ), UINT32_C( 250000 ), UINT32_C( 125000 )
			};

			var1 = (int64_t)(( 1340 + ( 5 * (int64_t)mysensor->caldata.range_sw_err )) * ( (int64_t)lookup_table1[gas_range] )) >> 16;
			var2 = (( (int64_t)( (int64_t)gas_res_adc << 15 ) - (int64_t)( 16777216 )) + var1);
			var3 = (( (int64_t)lookup_table2[gas_range] * (int64_t)var1 ) >> 9);
			calc_gas_res = (uint32_t)(( var3 + ( (int64_t)var2 >> 1 )) / (int64_t)var2 );

			return calc_gas_res;
		}

		static uint32_t calc_gas_resistance_high( uint16_t gas_res_adc, uint8_t gas_range )
		{
			uint32_t calc_gas_res;
			uint32_t var1 = UINT32_C( 262144 ) >> gas_range;
			int32_t var2 = (int32_t)gas_res_adc - INT32_C( 512 );

			var2 *= INT32_C( 3 );
			var2 = INT32_C( 4096 ) + var2;

			/// multiplying 10000 then dividing then multiplying by 100 instead of multiplying by 1000000 to prevent overflow
			calc_gas_res = ( UINT32_C( 10000 ) * var1 ) / (uint32_t)var2;
			calc_gas_res = calc_gas_res * 100;

			return calc_gas_res;
		}

		static uint8_t calc_res_heat( struct BME680_Instance * mysensor, uint16_t temp )
		{
			uint8_t heatr_res;
			int32_t var1;
			int32_t var2;
			int32_t var3;
			int32_t var4;
			int32_t var5;
			int32_t heatr_res_x100;
			
			if (temp > 400) 	/// Cap temperature
				temp = 400;
			
			var1 = (( (int32_t)mysensor->amb_temp * mysensor->caldata.par_gh3 ) / 1000 ) * 256;
			var2 = ( mysensor->caldata.par_gh1 + 784 ) * ((((( mysensor->caldata.par_gh2 + 154009 ) * temp * 5 ) / 100 ) + 3276800 ) / 10 );
			var3 = var1 + ( var2 / 2 );
			var4 = ( var3 / ( mysensor->caldata.res_heat_range + 4 ));
			var5 = ( 131 * mysensor->caldata.res_heat_val ) + 65536;
			heatr_res_x100 = (int32_t)((( var4 / var5 ) - 250 ) * 34 );
			heatr_res = (uint8_t)(( heatr_res_x100 + 50 ) / 100 );
			
			return heatr_res;
		}
	#endif	/*ENABLE_GAS_MEAS*/

#else	/// #ifdef BME680_RETURNS_FLOAT 
	static float calc_temperature( struct BME680_Instance * mysensor, uint32_t temp_adc )
	{
		float var1;
		float var2;
		float calc_temp;
	//	float t_fine;
		
		var1 = ((( (float)temp_adc / 16384.0f ) - ( (float)mysensor->caldata.par_t1 / 1024.0f )) * ( (float)mysensor->caldata.par_t2 ));
		var2 = (((( (float)temp_adc / 131072.0f ) - ( (float)mysensor->caldata.par_t1 / 8192.0f )) *
			(( (float)temp_adc / 131072.0f ) - ( (float)mysensor->caldata.par_t1 / 8192.0f ))) * ( (float)mysensor->caldata.par_t3 * 16.0f ));
		
		/* t_fine value*/
		mysensor->caldata.t_fine = ( var1 + var2 );
		
		/* compensated temperature data*/
		calc_temp = (( mysensor->caldata.t_fine ) / 5120.0f );
		
		return calc_temp;
	}

	static float calc_pressure( struct BME680_Instance * mysensor, uint32_t pres_adc )
	{
		float var1;
		float var2;
		float var3;
		float calc_pres;
	
		var1 = (( (float)mysensor->caldata.t_fine / 2.0f ) - 64000.0f );
		var2 = var1 * var1 * (( (float)mysensor->caldata.par_p6 ) / ( 131072.0f ));
		var2 = var2 + ( var1 * ( (float)mysensor->caldata.par_p5 ) * 2.0f );
		var2 = ( var2 / 4.0f ) + (( (float)mysensor->caldata.par_p4 ) * 65536.0f );
		var1 = (((( (float)mysensor->caldata.par_p3 * var1 * var1 ) / 16384.0f ) + ( (float)mysensor->caldata.par_p2 * var1 )) / 524288.0f );
		var1 = (( 1.0f + ( var1 / 32768.0f )) * ( (float)mysensor->caldata.par_p1 ));
		calc_pres = ( 1048576.0f - ( (float)pres_adc ));
	
		/* Avoid exception caused by division by zero */
		if ( (int)var1 != 0 )
		{
			calc_pres = ((( calc_pres - ( var2 / 4096.0f )) * 6250.0f ) / var1 );
			var1 = (( (float)mysensor->caldata.par_p9 ) * calc_pres * calc_pres ) / 2147483648.0f;
			var2 = calc_pres * (( (float)mysensor->caldata.par_p8 ) / 32768.0f );
			var3 = (( calc_pres / 256.0f ) * ( calc_pres / 256.0f ) * ( calc_pres / 256.0f ) * ( mysensor->caldata.par_p10 / 131072.0f ));
			calc_pres = ( calc_pres + ( var1 + var2 + var3 + ( (float)mysensor->caldata.par_p7 * 128.0f )) / 16.0f );
		}
		else
			calc_pres = 0;

		return calc_pres;
	}

	static float calc_humidity( struct BME680_Instance * mysensor, uint16_t hum_adc )
	{
		float calc_hum;
		float var1;
		float var2;
		float var3;
		float var4;
		float temp_comp;

		/* compensated temperature data*/
		temp_comp = (( mysensor->caldata.t_fine ) / 5120.0f );
		var1 = (float)( (float)hum_adc ) - (( (float)mysensor->caldata.par_h1 * 16.0f ) + (( (float)mysensor->caldata.par_h3 / 2.0f ) * temp_comp ));
		var2 = var1 * ( (float)(( (float)mysensor->caldata.par_h2 / 262144.0f ) * ( 1.0f + (( (float)mysensor->caldata.par_h4 / 16384.0f ) * temp_comp ) +
				(( (float)mysensor->caldata.par_h5 / 1048576.0f ) * temp_comp * temp_comp ))));
		var3 = (float)mysensor->caldata.par_h6 / 16384.0f;
		var4 = (float)mysensor->caldata.par_h7 / 2097152.0f;

		calc_hum = var2 + (( var3 + ( var4 * temp_comp )) * var2 * var2 );
		if ( calc_hum > 100.0f )
			calc_hum = 100.0f;
		else if ( calc_hum < 0.0f )
			calc_hum = 0.0f;

		return calc_hum;
	}

	#ifdef ENABLE_GAS_MEAS
		static float calc_gas_resistance_low( struct BME680_Instance * mysensor, uint16_t gas_res_adc, uint8_t gas_range )
		{
			float calc_gas_res;
			float var1;
			float var2;
			float var3;
			float gas_res_f = gas_res_adc;
			float gas_range_f = ( 1U << gas_range );

			const float lookup_k1_range[16] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f };
			const float lookup_k2_range[16] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

			var1 = ( 1340.0f + ( 5.0f * mysensor->caldata.range_sw_err ));
			var2 = ( var1 ) * ( 1.0f + lookup_k1_range[gas_range] / 100.0f );
			var3 = 1.0f + ( lookup_k2_range[gas_range] / 100.0f );
			calc_gas_res = 1.0f / (float)( var3 * ( 0.000000125f ) * gas_range_f * ((( gas_res_f - 512.0f ) / var2 ) + 1.0f ));

			return calc_gas_res;
		}

		static float calc_gas_resistance_high( uint16_t gas_res_adc, uint8_t gas_range )
		{
			float calc_gas_res;
			uint32_t var1 = UINT32_C( 262144 ) >> gas_range;
			int32_t var2 = (int32_t)gas_res_adc - INT32_C( 512 );

			var2 *= INT32_C( 3 );
			var2 = INT32_C( 4096 ) + var2;

			calc_gas_res = 1000000.0f * (float)var1 / (float)var2;
			return calc_gas_res;
		}

		static uint8_t calc_res_heat( struct BME680_Instance * mysensor, uint16_t temp )
		{
			float var1;
			float var2;
			float var3;
			float var4;
			float var5;
			uint8_t res_heat;

			if (temp > 400) /* Cap temperature */
			{
				temp = 400;
			}

			var1 = (((float)mysensor->caldata.par_gh1 / (16.0f)) + 49.0f);
			var2 = ((((float)mysensor->caldata.par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
			var3 = ((float)mysensor->caldata.par_gh3 / (1024.0f));
			var4 = (var1 * (1.0f + (var2 * (float)temp)));
			var5 = (var4 + (var3 * (float)dev->amb_temp));
			res_heat =
				(uint8_t)(3.4f *
						((var5 * (4 / (4 + (float)mysensor->caldata.res_heat_range)) *
							(1 / (1 + ((float)mysensor->caldata.res_heat_val * 0.002f)))) -
						25));

			return res_heat;
		}
	#endif /*ENABLE_GAS_MEAS*/

#endif /// BME680_RETURNS_FLOAT


///===================================================================
uint8_t BME680_GetResults( struct BME680_Instance * mysensor, struct BME680_ADCData * adcdata )
{
	uint8_t _rdbyte[17];
//	uint8_t retries = RETRY_COUNT;
	uint8_t vmult;


	adcdata->status = 0U;	/// Reset ADC Read status to 0; this goes 1 when we have a valid read
	//while( retries && !( adcdata->status ))
	//{
		/// Read addr 0x1D to 0x2D (17 reads)
		bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_MEAS_STATUS_0_BASE, 17U, _rdbyte );
		
		/// Check if new data is ready, and no measurements are being made
		adcdata->status = (_rdbyte[0] & ( MEAS_STATUS_0_NEW_DATA_0_Mask | MEAS_STATUS_0_GAS_MEASURING_Mask | MEAS_STATUS_0_MEASURING_Mask) ) == MEAS_STATUS_0_NEW_DATA_0_Mask ? BME680_OK : BME680_ERR;

		if( adcdata->status )
		{
			adcdata->meas_index = _rdbyte[1];		/// Store data index
			/// Begin organizing the ADC data:
			
			adcdata->temp_adc /* MSB:LSB = 0x22 / 0x23 / 0x24[7:4] */ = 
				((uint32_t)(_rdbyte[5] & TEMP_MSB_Mask ) << ( 8U + TEMP_XLSB_Pos )) |
				((uint32_t)(_rdbyte[6] & TEMP_LSB_Mask ) << TEMP_XLSB_Pos ) |
				((_rdbyte[7] & TEMP_XLSB_Mask ) >> TEMP_XLSB_Pos );

			adcdata->pres_adc /* MSB:LSB = 0x1F / 0x20 / 0x21[7:4] */ =
				((uint32_t)(_rdbyte[2] & PRESS_MSB_Mask ) << ( 8U + PRESS_XLSB_Pos )) |
				((uint32_t)(_rdbyte[3] & PRESS_LSB_Mask ) << PRESS_XLSB_Pos ) |
				((_rdbyte[4] & PRESS_XLSB_Mask ) >> PRESS_XLSB_Pos );

			adcdata->hum_adc /* MSB:LSB = 0x25 / 0x26 */ = ((uint16_t)(_rdbyte[8] & HUM_MSB_Mask ) << 8U ) | _rdbyte[9];

			#ifdef ENABLE_GAS_MEAS
				if( mysensor->run_gas )
				{
					/// Check if variant is "low" or "high"; and use the _rdbyte[] accordingly
					vmult = mysensor->variant_id == BME680_VARIANT_HIGH ? 2U : 0U;
					
					/// Read addr 0x2A-0x2D
					bme680_i2c_read( mysensor->i2c_channel, BME680_ADDR_GAS_BASE, 4U, _rdbyte );

					/// Store relevant data from low/high register
					adcdata->gas_range = _rdbyte[1 + vmult] & GAS_R_LSB_GAS_RANGE_R_Mask;
					adcdata->gas_res_adc = ((uint16_t)_rdbyte[1 + vmult] << 2 )| ((_rdbyte[0 + vmult] & GAS_R_LSB_GAS_R_Mask ) >> 6 );
					
					/// Get GAS_VALID status
					adcdata->status |= (_rdbyte[1 + vmult] & GAS_R_LSB_GAS_VALID_R_Mask ) >> GAS_R_LSB_GAS_VALID_R_Pos;
					adcdata->status |= (_rdbyte[1 + vmult] & GAS_R_LSB_HEAT_STAB_R_Mask ) >> GAS_R_LSB_HEAT_STAB_R_Pos;
				}
			#endif

			if( adcdata->status )
			{
				#ifndef BME680_RETURNS_FLOAT
					adcdata->temperature = calc_temperature( mysensor, adcdata->temp_adc );
					adcdata->pressure = calc_pressure( mysensor, adcdata->pres_adc );
					adcdata->humidity = calc_humidity( mysensor, adcdata->hum_adc );
				
					#ifdef ENABLE_GAS_MEAS
						if( mysensor->run_gas )
						{
							if( mysensor->variant_id == BME680_VARIANT_LOW )
								adcdata->gas_resistance = calc_gas_resistance_low( mysensor, adcdata->gas_res_adc, adcdata->gas_range );
							else
								adcdata->gas_resistance = calc_gas_resistance_high( adcdata->gas_res_adc, adcdata->gas_range );
						}
					#endif
				#else
					adcdata->temperature = calc_temperature( mysensor, adcdata->temp_adc );
					adcdata->pressure = calc_pressure( mysensor, adcdata->pres_adc );
					adcdata->humidity = calc_humidity( mysensor, adcdata->hum_adc );

					#ifdef ENABLE_GAS_MEAS
						if( mysensor->run_gas )
						{
							if( mysensor->variant_id == BME680_VARIANT_HIGH )
								adcdata->gas_resistance = calc_gas_resistance_low( mysensor, adcdata->gas_res_adc, adcdata->gas_range );
							else
								adcdata->gas_resistance = calc_gas_resistance_high( adcdata->gas_res_adc, adcdata->gas_range );
						}
					#endif
				#endif
			}
		}	
	//}
	return adcdata->status;
}