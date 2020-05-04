#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "defines.h"
#include "macros.h"
#include "onewire.h"
#include "ds18b20.h"
#include "radio.h"
#include "utils.h"

#ifdef SERIAL
#include <stdio.h>
#include "serial.h"
#endif


ISR(WDT_vect)
{
	wdt_disable();
	
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = 0x00;
}

void sleep2(uint8_t long_sleep)
{
	MCUSR = 0;
	
	if (long_sleep)
	{
		// enable interrupt, set timer to 8 sec
		WDTCR |= _BV(WDIE) | _BV(WDP3) | _BV(WDP0);
	}
	else
	{
		// enable interrupt, set timer to 1 sec
		WDTCR |= _BV(WDIE) | _BV(WDP2) | _BV(WDP1);
	}
	
	// enable watchdog
	WDTCR |= _BV(WDCE) | _BV(WDE);
	
	wdt_reset();
	
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
	
	wdt_disable();
}



inline void init()
{
	wdt_disable();
	
	DDR = 0x00 | _BV(PIN_RADIO) | _BV(PIN_LED);
	
#ifdef SERIAL
	USART_Init();
	
	USART_TransmitString("Hello!\r\n");
#endif
}


int main()
{
#ifdef SERIAL
	char s[50];
#endif
	
	int16_t reading, temperature10;
	uint16_t a1;
	uint8_t a2, a3;
	
	init();
	
	while (1)
	{
		LED_ON;
		_delay_ms(50);
		LED_OFF;
		
		_delay_ms(100);
		
		if (onewire_reset())
		{
			LED_ON;
			_delay_ms(50);
			LED_OFF;
			
			onewire_search_init();
			
			// loop through all devices
			while (onewire_search())
			{
				if (!onewire_check_rom_crc())
				{
#ifdef SERIAL
					USART_TransmitString("check_rom: crc error");
#endif
					continue;
				}
				
				// // read the temperature
				// ds18b20_convert_slave();
				// 
				// // wait for conversion to finish
				// _delay_ms(750);
				// 
				// // read the temperature from device
				// reading = ds18b20_read_slave();
				
				// read the temperature
				ds18b20_convert_slave();
				
				// wait for conversion to finish - 1 second
				sleep2(false);
				
				// read the temperature from device
				reading = ds18b20_read_slave();
				
				// if reading failed skip this device
				if (reading != kDS18B20_CrcCheckFailed)
				{
					a1 = onewire_get_address_hash();
					
					// *** prologue and solight***
					// calculate a radio device id and channel
					// that are valid for the emulated thermometer type
					
					a2 = (a1 >> 4) & 0x0F;
					a3 = a1 & 0x03 + 1;
					
					
					// Convert to floating point (or keep as a Q12.4 fixed point value)
					// float temperature = ((float) reading) / 16;
					
					// convert to integer, expressed in 0.1 deg C
					temperature10 = reading * 10 / 16;
					
#ifdef SERIAL
					// send the device index, generated device id and channel, address on serial
					sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%02x (%04x, id: %02x, ch: %02x) ", ow_search_address[0], ow_search_address[1], ow_search_address[2], ow_search_address[3], ow_search_address[4], ow_search_address[5], ow_search_address[6], ow_search_address[7], a1, a2, a3);
					USART_TransmitString(s);
					
					sprintf(s, "%d (0x%04x)", temperature10, reading);
					USART_TransmitString(s);
#endif
					
					// void prologue_send(uint8_t id, uint8_t channel, int16_t temperature10, uint8_t humidity, uint8_t battery_status, uint8_t button_pressed)
					// prologue_send(1, 2, 123, 45, 1, 0);
					// prologue_send(a2, a3, temperature10, 11, 1, 0);
					
					solight_te44_send(a2, a3, temperature10);
					
					// wait before going to the next device - 64 seconds
					sleep2(true);
					sleep2(true);
					sleep2(true);
					sleep2(true);
					sleep2(true);
					sleep2(true);
					sleep2(true);
					sleep2(true);
					
#ifdef SERIAL
					USART_TransmitString("\r\n");
#endif
				}
				else
				{
#ifdef SERIAL
					USART_TransmitString("read_slave: crc error\r\n");
#endif
					continue;
				}
			}
			
#ifdef SERIAL
			USART_TransmitString("\r\n");
#endif
		}
	}
	
	return 0;
}
