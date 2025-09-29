# CLIMATE CONTROL PROJECT:

## Description
   	* For Bosch BME680
		* I2C communication
		* Integer-based (STM32F1); disabled floating-point equations
		* Disabled Gas Sensor functionality - not enough SRAM

## Resources Used:
		* TIM2 - wait delays
		* I2C1 - PB6 SCL; PB7 SDA; or...
		* I2C2 - PB10 SCL; PB11 SDA

## Hardware:
		* [BME680 Module](https://www.lazada.com.ph/products/i4670716201-s26920552349.html)
		* [SH1106 1.3" OLED Display](https://www.lazada.com.ph/products/circuitrocks-oled-i2c-module-lcd-display-screen-09109613-inch-oled-communicate-for-arduino-i111661352-s7936659215.html)
		* [Relay Module](https://www.lazada.com.ph/products/i100047427-s100061336.html)

## To do:
		* Add controls for relay, based on Temp/Hum limits
		* Upload MBSE documentation
