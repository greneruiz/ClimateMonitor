CLIMATE CONTROL PROJECT:

/// Description
///		* For Bosch BME680
///		* I2C communication
///		* Integer-based (STM32F1); disabled floating-point equations
///		* Disabled Gas Sensor functionality - not enough SRAM
///		* To do (v1.0): Add controls for relay based on Temp/Hum limits; upload MBSE documentation
///
///	Resources Used:
///		* TIM2 - wait delays
///		* I2C1 - PB6 SCL; PB7 SDA; or...
///		* I2C2 - PB10 SCL; PB11 SDA
///
/// Hardware:
///		* BME680 Module
///		* SH1106 1.3" OLED Display
