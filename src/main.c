///===================================================================
/// File Name: main.c
/// Type     : STM32 C-source
/// Purpose  : Driver for Climate Control System
/// Version  : 1.0
///===================================================================
/// Description
///		* For Bosch BME680
///		* I2C communication
///		* Integer-based (STM32F1); disabled floating-point equations
///		* Disabled Gas Sensor functionality - not enough SRAM
///		* To do: Add controls for relay based on Temp/Hum limits
///
///	Resources Used:
///		* TIM2 - wait delays
///		* I2C1 - PB6 SCL; PB7 SDA; or...
///		* I2C2 - PB10 SCL; PB11 SDA
///
/// Hardware:
///		* BME680 Module
///		* SH1106 1.3" OLED Display
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Sep-29 / G.RUIZ
///		* Initial release
///===================================================================


#include <stdio.h>
#include <stdint.h>
//#include <stdlib.h>


//#include "ggg_usart_debug.h"
#include "BME680Ctrl.h"
#include "SH1106.h"
#include "GGG_GFX.h"

#include "font_5x7.h"


/// @brief array sizes
const uint8_t c_ltemp_size = 6U;
const uint8_t c_lpres_size = 6U;
const uint8_t c_lhumi_size = 6U;

const uint16_t d_temph_origin[2] = { 13U, 3U };
const uint16_t d_tempf_origin[2] = { (d_temph_origin[0] + ( font_5x7_width * 14U )), d_temph_origin[1] };
const uint16_t d_tempm_origin[2] = { ( d_temph_origin[0] + ( font_5x7_width * 8U )), d_temph_origin[1] };

const uint16_t d_presh_origin[2] = { d_temph_origin[0], ( d_temph_origin[1] + 1U + font_5x7_height ) };
const uint16_t d_presf_origin[2] = { (d_temph_origin[0] + ( font_5x7_width * 14U )), d_presh_origin[1] };
const uint16_t d_presm_origin[2] = { ( d_presh_origin[0] + ( font_5x7_width * 8U ) ), d_presh_origin[1] };

const uint16_t d_humih_origin[2] = { d_temph_origin[0], ( d_presh_origin[1] + 1U + font_5x7_height ) };
const uint16_t d_humif_origin[2] = { (d_temph_origin[0] + ( font_5x7_width * 14U )), d_humih_origin[1] };
const uint16_t d_humim_origin[2] = { ( d_humih_origin[0] + ( font_5x7_width * 8U ) ), d_humih_origin[1] };


/// @brief Convert int16_t temp to character array of 6 (sign + 2 digits + '.' + 2 decimals)
/// Character array is organized first character at MSindex
/// @param input int16_t to be converted
/// @param arr pass-by-reference to character array of size 7
void temp_to_carray( int16_t input, char * arr )
{
	/// initialize in case input is not in the 100's
	arr[0] = ' ';
	arr[1] = ' ';
	arr[2] = ' ';

	uint8_t ptr = c_ltemp_size - 1U;
	/// Check if input is negative
	if( input & 0x8000 )
	{
		arr[0] = '-';
		input = -input;	/// make input positive
	}
	else
		arr[0] = ' ';

	while( ptr > 0 )
	{
		if( ptr == 3 )
		{
			arr[ptr] = '.';
		}
		else
		{
			arr[ptr] = ( input % 10 ) + '0';
			input /= 10;
		}
		ptr--;	
	}
}

/// @brief Convert uint32_t pressure to character array of 6
/// @param input uint32_t to be converted
/// @param arr pass-by-reference to character array of size 6
void pres_to_carray( uint32_t input, char * arr )
{
	/// initialize in case input is not in the 100's
	arr[0] = ' ';
	arr[1] = ' ';
	arr[2] = ' ';

	uint8_t ptr = c_lpres_size - 1U;
	while( input > 0 )
	{
		arr[ptr] = ( input % 10 ) + '0';
		input /= 10;
		ptr--;
	}
}

void hum_to_carray( uint32_t input, char * arr )
{
	/// initialize in case input is not in the 100's
	arr[0] = ' ';
	arr[1] = ' ';
	arr[2] = ' ';

	uint8_t ptr = c_lhumi_size - 1U;
	while( input > 0 )
	{
		if( ptr == 2 )
		{
			arr[ptr] = '.';
		}
		else
		{
			arr[ptr] = ( input % 10 ) + '0';
			input /= 10;
		}
		ptr--;
	}
}


typedef enum
{
	measTemp,
	measPres,
	measHumi,
	measGasR
} measType;

void assemble_header_footer( measType type, char * head, char * foot )
{
	switch( type )
	{
		case measTemp:
			head[0] = ' ';
			head[1] = 'T';
			head[2] = 'e';
			head[3] = 'm';
			head[4] = 'p';
			head[5] = ' ';
			head[6] = '=';
			head[7] = ' ';
			foot[0] = ' ';
			foot[1] = 'C';
			foot[2] = ' ';
			break;
		case measPres:
			head[0] = ' ';
			head[1] = 'P';
			head[2] = 'r';
			head[3] = 'e';
			head[4] = 's';
			head[5] = ' ';
			head[6] = '=';
			head[7] = ' ';
			foot[0] = ' ';
			foot[1] = 'P';
			foot[2] = 'a';
			break;
		case measHumi:
			head[0] = ' ';
			head[1] = 'H';
			head[2] = 'u';
			head[3] = 'm';
			head[4] = ' ';
			head[5] = ' ';
			head[6] = '=';
			head[7] = ' ';
			foot[0] = ' ';
			foot[1] = '%';
			foot[2] = ' ';
			break;
		default:
			head[0] = ' ';
			head[1] = 'G';
			head[2] = 'a';
			head[3] = 's';
			head[4] = 'R';
			head[5] = ' ';
			head[6] = '=';
			head[7] = ' ';
			foot[0] = 'O';
			foot[1] = 'h';
			foot[2] = 'm';
	};
}


int main( void )
{
	struct BME680_Instance sensor1;
	struct BME680_ADCData meas1;
	struct SH1106_Instance display;

	/// Prepare display:
	SH1106_Sample_Setup( &display );
	SH1106_Initialize( &display );
	
	/// Print out the output template:
	char header[8];
	char footer[3];

	char ltemp[c_ltemp_size];
	char lpres[c_lpres_size];
	char lhumi[c_lhumi_size];


	/// Temp = +31.59 C
	assemble_header_footer( measTemp, header, footer );
	gfx_print_string( d_temph_origin[0], d_temph_origin[1], header, 7U, font_5x7_id, &display );
	gfx_print_string ( d_tempf_origin[0], d_tempf_origin[1], footer, 3U, font_5x7_id, &display );

	/// Pres = 101387 Pa
	assemble_header_footer( measPres, header, footer );
	gfx_print_string( d_presh_origin[0], d_presh_origin[1], header, 7U, font_5x7_id, &display );
	gfx_print_string ( d_presf_origin[0], d_presf_origin[1], footer, 3U, font_5x7_id, &display );

	/// Hum  = 73.101 %
	assemble_header_footer( measHumi, header, footer );
	gfx_print_string( d_humih_origin[0], d_humih_origin[1], header, 7U, font_5x7_id, &display );
	gfx_print_string ( d_humif_origin[0], d_humif_origin[1], footer, 3U, font_5x7_id, &display );


	/// Prepare sensor1:
	BME680_Sample_Sensor_Setup( &sensor1 );
	BME680_Initialize( &sensor1 );
	BME680_Set_Oversampling_and_Filter( &sensor1 );
#ifdef ENABLE_GAS_MEAS
	if( mysensor->run_gas ) BME680_Set_GasHeater( &sensor1 );	
#endif



	while(1)
	{

		init_tim2( MEASDLY_PRESCALER, MEASDLY_CNT_100MS );
		tim2_wait_delay();
		BME680_RunMeas( &sensor1 );

		init_tim2( MEASDLY_PRESCALER, MEASDLY_CNT_100MS );
		tim2_wait_delay();
		BME680_GetResults( &sensor1, &meas1 );

		temp_to_carray( meas1.temperature, ltemp );
		gfx_print_string( d_tempm_origin[0], d_tempm_origin[1], ltemp, c_ltemp_size, font_5x7_id, &display );

		pres_to_carray( meas1.pressure, lpres );
		gfx_print_string( d_presm_origin[0], d_presm_origin[1], lpres, c_lpres_size, font_5x7_id, &display );

		hum_to_carray( meas1.humidity, lhumi );
		gfx_print_string( d_humim_origin[0], d_humim_origin[1], lhumi, c_lhumi_size, font_5x7_id, &display );

		/// Setup meas delay: 3 seconds: preload 10kHz, count 2400 (3sec)
		init_tim2( 0x270F, 0x095F );
		tim2_wait_delay();
	}
}