#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 8000000UL // 8 MHz clock speed
#endif

// #define SERIAL 1

#define DDR DDRB
#define PORT PORTB
#define PIN_IN PINB
#define PIN_RADIO 3
#define PIN_LED 1
#define PIN_SENSOR 4

#define OW_DDR DDR
#define OW_PORT PORT
#define OW_PIN PIN_IN
#define OW_BIT PIN_SENSOR
