
#define STM32F103xB
#include "stm32f1xx.h"


void tim2_wait_delay( void )
{
	/// The actual delay part:
	while( !READ_BIT( TIM2->SR, TIM_SR_UIF )){}
	CLEAR_BIT( TIM2->SR, TIM_SR_UIF );
}

void init_tim2( uint16_t prescaler_value, uint16_t reload_value )
{
	/// Counter disable
	CLEAR_BIT( TIM2->CR1, TIM_CR1_CEN );

	/// Enable clock for TIM2
	SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM2EN );

	/// Set prescaler: 8MHz / prescaler_value - 1
	WRITE_REG( TIM2->PSC, ( prescaler_value & TIM_PSC_PSC_Msk ));

	/// Set auto-reload value
	WRITE_REG( TIM2->ARR, ( reload_value & TIM_ARR_ARR_Msk ));

	/// Clear counter
	WRITE_REG( TIM2->CNT, 0x0000UL );

	/// Counter enable
	SET_BIT( TIM2->CR1, TIM_CR1_CEN );
}