
#ifndef __WAITDLY_H__
#define __WAITDLY_H__


/// @brief Using TIM2, set prescaler and autoreload value for the delay.
/// @param prescaler_value	Prescaler Frequency in Hz. Hex value minus one.
/// @param reload_value		Autoreload value minus one
/// Formula for reload_value = ( Sysclk_hz / (prescaler_value * target_delay_freq_hz )) - 1
void init_tim2( uint16_t prescaler_value, uint16_t reload_value );

/// @brief Execute the wait delay programmed on TIM2 using init_tim2() 
void tim2_wait_delay( void );

#endif /*__WAITDLY_H__*/