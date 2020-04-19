# dsblip

Send the measurements of one or more DS1820/DS18S20/DS128B20 1-wire thermometers over 433 MHz radio using an AVR microcontroller.

Components:
  - AVR microcontroller
  - DS18* 1-wire thermometer
  - XKFST radio transmitter
  - an LED

The code uses 3 I/O pins for:
  - communicating with the thermometers
  - sending the data through the radio
  - driving a status LED

The final size is about 1700 bytes.


## Thanks

The project is using previous works of others, so thank you for:
  - stecman for 1-wire and DS1820: https://gist.github.com/stecman/9ec74de5e8a5c3c6341c791d9c233adc
  - the ATMEL docs for serial communication: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
