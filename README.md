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
