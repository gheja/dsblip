#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>

#include "radio.h"

#include "defines.h"
#include "macros.h"


static void send_ppm(uint8_t data[], uint8_t length, uint8_t repeats, uint8_t solight_mode)
{
	uint8_t i, k;
	
	for (k=0; k<repeats; k++)
	{
		// sync pulse between the repeats
		if (k != 0)
		{
			RADIO_ON;
			_delay_us(PPM_TIME_PULSE);
			
			RADIO_OFF;
			_delay_us(PPM_TIME_PULSE);
			
			RADIO_OFF;
			_delay_us(PPM_TIME_SYNC);
		}
		
		for (i=0; i<length; i++)
		{
			RADIO_ON;
			_delay_us(PPM_TIME_PULSE);
			
			RADIO_OFF;
			
			// send the next bit
			if (data[i / 8] & (1 << (7 - (i % 8))))
			{
				_delay_us(PPM_TIME_OFF_1);
			}
			else
			{
				_delay_us(PPM_TIME_OFF_0);
			}
		}
		
		// final bit
		
		RADIO_ON;
		_delay_us(PPM_TIME_PULSE);
		
		RADIO_OFF;
		_delay_us(PPM_TIME_PULSE);
	}
}

static void prologue_send(uint8_t id, uint8_t channel, int16_t temperature10, uint8_t humidity, uint8_t battery_status, uint8_t button_pressed)
{
	uint8_t data[5];
	uint8_t length;
	
	// RTL-433 recognises this device as "Prologue sensor", with id 9.
	// NOTE: RTL-433 will ignore the device if all these conditions meet:
	//   - battery_status = 0 
	//   - button_pressed = 0
	//   - channel = 1
	
	// Domoticz recognises this device as the following:
	//   ID: 9xxy, where xx is "id" in hex, y is "channel" - 1 in hex
	//   Name: Prologue
	//   Type: Temp + Humidity
	//   SubType: WTGR800
	
	// identifier (0-255)
	// NOTE: Domoticz ignores ids outside 1-127
	
	// channel (1-4)
	// battery status (0: LOW, 1: OK)
	// measurements sent by pressing button (not automatic)
	// temperature (signed integer (two's complement), celsius x 10)
	// humidity (percent, 0-100)
	
	data[0] = 0b10010000;
	data[1] = 0b00000000;
	data[2] = 0b00000000;
	data[3] = 0b00000000;
	data[4] = 0b00000000;
	
	length = 37;
	
	data[0] |= (id & 0xF0) >> 4;
	data[1] |= (id & 0x0F) << 4;
	data[1] |= (battery_status & 0x01) << 3;
	data[1] |= (button_pressed & 0x04) << 2;
	data[1] |= ((channel - 1) & 0x03);
	data[2] |= (temperature10  & 0x0FF0) >> 4;
	data[3] |= (temperature10 & 0x000F) << 4;
	data[3] |= (humidity & 0xF0) >> 4;
	data[4] |= (humidity & 0x0F) << 4;
	
	send_ppm(data, length, 7, 0);
}

// based on https://github.com/merbanan/rtl_433/blob/master/src/devices/rubicson.c - thanks!
static uint8_t rubicson_crc(uint8_t const data[])
{
	uint8_t remainder, byte, bit;
	
	const uint8_t length = 4;
	const uint8_t polynomial = 0x31;
	
	// initial value for remainder
	remainder = 0x6c;
	
	for (byte = 0; byte < length; byte++)
	{
		remainder ^= data[byte];
		
		for (bit = 0; bit < 8; bit++)
		{
			if (remainder & 0x80)
			{
				remainder = (remainder << 1) ^ polynomial;
			}
			else
			{
				remainder = (remainder << 1);
			}
		}
	}
	
	return remainder;
}

void solight_te44_send(uint8_t id, uint8_t channel, int16_t temperature10)
{
	uint8_t data[5];
	uint8_t crc;
	
	data[0] = 0b00000000;
	data[1] = 0b10000000;
	data[2] = 0b00000000;
	data[3] = 0b11110000;
	data[4] = 0b00000000;
	
	data[0] |= id;
	data[1] |= ((channel - 1) & 0x04) << 4;
	data[1] |= (temperature10 & 0x0F00) >> 8;
	data[2] |= temperature10 & 0x00FF;
	
	// test data
/*
	data[0] = 0b00100110;
	data[1] = 0b10000000;
	data[2] = 0b00101111;
	data[3] = 0b11110000;
	data[4] = 0b00000000;
*/
	
	crc = rubicson_crc(data);
	
	data[3] |= (crc & 0xf0) >> 4;
	data[4] |= (crc & 0x0f) << 4;
	
	send_ppm(data, 36, 12, 1);
	
	// printf("%d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4]);
}
