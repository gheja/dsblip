#ifndef F_CPU
#define F_CPU 8000000UL // 8 MHz clock speed
#endif

// #define SERIAL 1

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>

#ifdef SERIAL
#include <stdio.h>
#endif

// #define DDR DDRC
// #define PORT PORTC
// #define PIN_IN PINC
// #define PIN_RADIO 0
// #define PIN_LED 1
// #define PIN_SENSOR 2

#define DDR DDRB
#define PORT PORTB
#define PIN_IN PINB
#define PIN_RADIO 0
#define PIN_LED 1
#define PIN_SENSOR 2

#define OW_DDR DDR
#define OW_PORT PORT
#define OW_PIN PIN_IN
#define OW_BIT PIN_SENSOR

#define BIT_SET(a, b) a |= _BV(b)
#define BIT_CLEAR(a, b) a &= ~_BV(b)

#define RADIO_ON BIT_SET(PORT, PIN_RADIO)
#define RADIO_OFF BIT_CLEAR(PORT, PIN_RADIO)

#define LED_ON BIT_SET(PORT, PIN_LED)
#define LED_OFF BIT_CLEAR(PORT, PIN_LED)

#define RADIO_ON BIT_SET(PORT, PIN_RADIO)
#define RADIO_OFF BIT_CLEAR(PORT, PIN_RADIO)



inline void gset_output_high() {
    BIT_SET(OW_PORT, OW_BIT);
}

inline void gset_output_low() {
    BIT_CLEAR(OW_PORT, OW_BIT);
}

void gset_input_pullup() {
    OW_DDR &= ~_BV(OW_BIT);
    gset_output_high();
}

inline void gset_input_hiz() {
    BIT_CLEAR(OW_DDR, OW_BIT);
    gset_output_low();
}

inline void gset_output() {
    OW_DDR |= _BV(OW_BIT);
}

void gset_bit() {
    OW_PORT |= _BV(OW_BIT);
}

void gclear_bit() {
    OW_PORT &= ~_BV(OW_BIT);
}

inline uint8_t gread_bit() {
    return OW_PIN & _BV(OW_BIT);
}

uint8_t crc8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; ++i) {
        crc = _crc_ibutton_update(crc, data[i]);
    }

    return crc;
}





// The highest bit position where a bit was ambiguous and a zero was written
int8_t ow_search_lastZeroBranch;

// Internal flag to indicate if the search is complete
// This flag is set once there are no more branches to search
bool ow_search_done;

// Discovered 64-bit device address (LSB first)
// After a successful search, this contains the found device address.
// During a search this is overwritten LSB-first with a new address.
uint8_t ow_search_address[8];

inline void onewire_search_init()
{
    ow_search_lastZeroBranch = -1;
    ow_search_done = false;
    uint8_t i;

    // Zero-fill the address
    for (i=0; i<8; i++)
    {
        ow_search_address[i] = 0;
    }
}

bool onewire_reset()
{
    // Configure for output
    gset_output_high();
    gset_output();

    // Pull low for >480uS (master reset pulse)
    gset_output_low();
    _delay_us(480);

    // Configure for input
    gset_input_hiz();
    _delay_us(70);

    // Look for the line pulled low by a slave
    uint8_t result = gread_bit();

    // Wait for the presence pulse to finish
    // This should be less than 240uS, but the master is expected to stay
    // in Rx mode for a minimum of 480uS in total
    _delay_us(460);

    return result == 0;
}

/**
 * Output a Write-0 or Write-1 slot on the One Wire bus
 * A Write-1 slot is generated unless the passed value is zero
 */
void onewire_write_bit(uint8_t bit)
{
    if (bit != 0) { // Write high

        // Pull low for less than 15uS to write a high
        gset_output_low();
        _delay_us(5);
        gset_output_high();

        // Wait for the rest of the minimum slot time
        _delay_us(55);

    } else { // Write low

        // Pull low for 60 - 120uS to write a low
        gset_output_low();
        _delay_us(55);

        // Stop pulling down line
        gset_output_high();

        // Recovery time between slots
        _delay_us(5);
    }
}

// One Wire timing is based on this Maxim application note
// https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
void onewire_write(uint8_t byte)
{
    // Configure for output
    gset_output_high();
    gset_output();

    for (uint8_t i = 0; i<8; i++) {
        onewire_write_bit(byte & 1);
        byte >>= 1;
    }
}

/**
 * Generate a read slot on the One Wire bus and return the bit value
 * Return 0x0 or 0x1
 */
uint8_t onewire_read_bit()
{
    // Pull the 1-wire bus low for >1uS to generate a read slot
    gset_output_low();
    gset_output();
    _delay_us(1);

    // Configure for reading (releases the line)
    gset_input_hiz();

    // Wait for value to stabilise (bit must be read within 15uS of read slot)
    _delay_us(10);

    uint8_t result = gread_bit() != 0;

    // Wait for the end of the read slot
    _delay_us(50);

    return result;
}

uint8_t onewire_read()
{
    uint8_t buffer = 0x0;

    // Configure for input
    gset_input_hiz();

    // Read 8 bits (LSB first)
    for (uint8_t bit = 0x01; bit; bit <<= 1) {

        // Copy read bit to least significant bit of buffer
        if (onewire_read_bit()) {
            buffer |= bit;
        }
    }

    return buffer;
}

void onewire_match_rom()
{
    // Write Match Rom command on bus
    onewire_write(0x55);

    // Send the passed address
    for (uint8_t i = 0; i < 8; ++i) {
        onewire_write(ow_search_address[i]);
    }
}

void onewire_skiprom()
{
    onewire_write(0xCC);
}

/**
 * Search procedure for the next ROM addresses
 *
 * This algorithm is bit difficult to understand from the diagrams in Maxim's
 * datasheets and app notes, though its reasonably straight forward once
 * understood.  I've used the name "last zero branch" instead of Maxim's name
 * "last discrepancy", since it describes how this variable is used.
 *
 * A device address has 64 bits. With multiple devices on the bus, some bits
 * are ambiguous.  Each time an ambiguous bit is encountered, a zero is written
 * and the position is marked.  In subsequent searches at ambiguous bits, a one
 * is written at this mark, zeros are written after the mark, and the bit in
 * the previous address is copied before the mark. This effectively steps
 * through all addresses present on the bus.
 *
 * For reference, see either of these documents:
 *
 *  - Maxim application note 187: 1-Wire Search Algorithm
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/187
 *
 *  - Maxim application note 937: Book of iButton® Standards (pages 51-54)
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/937
 *
 * @see onewire_search()
 * @returns true if a new address was found
 */
bool _search_next()
{
    // States of ROM search reads
    enum {
        kConflict = 0b00,
        kZero = 0b10,
        kOne = 0b01,
    };

    // Value to write to the current position
    uint8_t bitValue = 0;

    // Keep track of the last zero branch within this search
    // If this value is not updated, the search is complete
    int8_t localLastZeroBranch = -1;

    for (int8_t bitPosition = 0; bitPosition < 64; ++bitPosition) {

        // Calculate bitPosition as an index in the address array
        // This is written as-is for readability. Compilers should reduce this to bit shifts and tests
        uint8_t byteIndex = bitPosition / 8;
        uint8_t bitIndex = bitPosition % 8;

        // Configure bus pin for reading
        gset_input_hiz();

        // Read the current bit and its complement from the bus
        uint8_t reading = 0;
        reading |= onewire_read_bit(); // Bit
        reading |= onewire_read_bit() << 1; // Complement of bit (negated)

        switch (reading) {
            case kZero:
            case kOne:
                // Bit was the same on all responding devices: it is a known value
                // The first bit is the value we want to write (rather than its complement)
                bitValue = (reading & 0x1);
                break;

            case kConflict:
                // Both 0 and 1 were written to the bus
                // Use the search state to continue walking through devices
                if (bitPosition == ow_search_lastZeroBranch) {
                    // Current bit is the last position the previous search chose a zero: send one
                    bitValue = 1;

                } else if (bitPosition < ow_search_lastZeroBranch) {
                    // Before the lastZeroBranch position, repeat the same choices as the previous search
                    bitValue = ow_search_address[byteIndex] & (1 << bitIndex);

                } else {
                    // Current bit is past the lastZeroBranch in the previous search: send zero
                    bitValue = 0;
                }

                // Remember the last branch where a zero was written for the next search
                if (bitValue == 0) {
                    localLastZeroBranch = bitPosition;
                }

                break;

            default:
                // If we see "11" there was a problem on the bus (no devices pulled it low)
                return false;
        }

        // Write bit into address
        if (bitValue == 0) {
            ow_search_address[byteIndex] &= ~(1 << bitIndex);
        } else {
            ow_search_address[byteIndex] |= (bitValue << bitIndex);
        }

        // Configure for output
        gset_output_high();
        gset_output();

        // Write bit to the bus to continue the search
        onewire_write_bit(bitValue);
    }

    // If the no branch points were found, mark the search as done.
    // Otherwise, mark the last zero branch we found for the next search
    if (localLastZeroBranch == -1) {
        ow_search_done = true;
    } else {
        ow_search_lastZeroBranch = localLastZeroBranch;
    }

    // Read a whole address - return success
    return true;
}

inline bool _search_devices(uint8_t command)
{
    // Bail out if the previous search was the end
    if (ow_search_done) {
        return false;
    }

    if (!onewire_reset()) {
        // No devices present on the bus
        return false;
    }

    onewire_write(command);
    return _search_next();
}

bool onewire_search()
{
    // Search with "Search ROM" command
    return _search_devices(0xF0);
}

bool onewire_alarm_search()
{
    // Search with "Alarm Search" command
    return _search_devices(0xEC);
}

bool onewire_check_rom_crc()
{
    // Validate bits 0..56 (bytes 0 - 6) against the CRC in byte 7 (bits 57..63)
    return ow_search_address[7] == crc8(ow_search_address, 7);
}




static const int8_t kScratchPadLength = 9;

// Special return values
static const uint16_t kDS18B20_DeviceNotFound = 0xA800;
static const uint16_t kDS18B20_CrcCheckFailed = 0x5000;

// Command bytes
static const uint8_t kConvertCommand = 0x44;
static const uint8_t kReadScatchPad = 0xBE;

// Scratch pad data indexes
static const uint8_t kScratchPad_tempLSB = 0;
static const uint8_t kScratchPad_tempMSB = 1;
static const uint8_t kScratchPad_tempCountRemain = 6;
static const uint8_t kScratchPad_crc = 8;

static uint16_t ds18b20_readScratchPad()
{
	// Read scratchpad into buffer (LSB byte first)
	static const int8_t kScratchPadLength = 9;
	uint8_t buffer[kScratchPadLength];
	
	for (int8_t i = 0; i < kScratchPadLength; ++i) {
		buffer[i] = onewire_read();
	}
	
	// Check the CRC (9th byte) against the 8 bytes of data
	if (crc8(buffer, 8) != buffer[kScratchPad_crc]) {
		return kDS18B20_CrcCheckFailed;
	}
	
	// Return the raw 9 to 12-bit temperature value
	return (buffer[kScratchPad_tempMSB] << 8) | buffer[kScratchPad_tempLSB];
}

uint16_t ds18b20_readScratchPad2()
{
	// Read scratchpad into buffer (LSB byte first)
	uint8_t buffer[kScratchPadLength];
	
	for (int8_t i = 0; i < kScratchPadLength; ++i) {
		buffer[i] = onewire_read();
	}
	
	// Check the CRC (9th byte) against the 8 bytes of data
	if (crc8(buffer, 8) != buffer[kScratchPad_crc]) {
		return kDS18B20_CrcCheckFailed;
	}
	
	// DS1820, DS18S20
	if (ow_search_address[0] == 0x10)
	{
		// temp_read - 0.25 + (count_per_c - count_remain) / count_per_c
		// count_per_c is always 16 (0x10)
		
		// Return the raw 9 to 12-bit temperature value
		return ((buffer[kScratchPad_tempLSB] >> 1) << 4) - 16 +
			(16 - (buffer[kScratchPad_tempCountRemain]));
	}
	else
	{
		// Return the raw 9 to 12-bit temperature value
		return (buffer[kScratchPad_tempMSB] << 8) | buffer[kScratchPad_tempLSB];
	}
}

uint16_t ds18b20_read_single()
{
	// Confirm the device is still alive. Abort if no reply
	if (!onewire_reset()) {
		return kDS18B20_DeviceNotFound;
	}
	
	// Reading a single device, so skip sending a device address
	onewire_skiprom();
	onewire_write(kReadScatchPad);
	
	// Read the data from the scratch pad
	return ds18b20_readScratchPad();
}

uint16_t ds18b20_read_slave()
{
	// Confirm the device is still alive. Abort if no reply
	if (!onewire_reset()) {
		return kDS18B20_DeviceNotFound;
	}
	
	onewire_match_rom();
	onewire_write(kReadScatchPad);
	
	// Read the data from the scratch pad
	return ds18b20_readScratchPad2();
}

uint16_t ds18b20_convert()
{
	// Confirm the device is still alive. Abort if no reply
	if (!onewire_reset()) {
		return kDS18B20_DeviceNotFound;
	}
	
	// Send convert command to all devices (this has no response)
	onewire_skiprom();
	onewire_write(kConvertCommand);
	
	return 0;
}

uint16_t ds18b20_convert_slave()
{
	// Confirm the device is still alive. Abort if no reply
	if (!onewire_reset()) {
		return kDS18B20_DeviceNotFound;
	}
	
	// Send convert command to the specified device (this has no response)
	onewire_match_rom();
	onewire_write(kConvertCommand);
	
	return 0;
}




/*
#define PPM_TIME_PULSE 583
#define PPM_TIME_OFF_0 2100
#define PPM_TIME_OFF_1 4020
#define PPM_TIME_SYNC 8650
*/

#define PPM_TIME_PULSE 500
#define PPM_TIME_OFF_0 990
#define PPM_TIME_OFF_1 1940
#define PPM_TIME_SYNC 3900

static void send_ppm(uint8_t bytes[], uint8_t length, uint8_t repeats, uint8_t solight_mode)
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
			if (bytes[i / 8] & (1 << (7 - (i % 8))))
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
	uint8_t bytes[5];
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
	
	bytes[0] = 0b10010000;
	bytes[1] = 0b00000000;
	bytes[2] = 0b00000000;
	bytes[3] = 0b00000000;
	bytes[4] = 0b00000000;
	
	length = 37;
	
	bytes[0] |= (id & 0xF0) >> 4;
	bytes[1] |= (id & 0x0F) << 4;
	bytes[1] |= (battery_status & 0x01) << 3;
	bytes[1] |= (button_pressed & 0x04) << 2;
	bytes[1] |= ((channel - 1) & 0x03);
	bytes[2] |= (temperature10  & 0x0FF0) >> 4;
	bytes[3] |= (temperature10 & 0x000F) << 4;
	bytes[3] |= (humidity & 0xF0) >> 4;
	bytes[4] |= (humidity & 0x0F) << 4;
	
	send_ppm(bytes, length, 7, 0);
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

static void solight_te44_send(uint8_t id, uint8_t channel, int16_t temperature10)
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



#ifdef SERIAL
void USART_Init()
{
	unsigned int ubrr = F_CPU / 16 / 9600 - 1;
	
	/* Set baud rate */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)));
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void USART_TransmitString(unsigned char a[])
{
	unsigned char i;
	
	for (i=0; a[i] != '\0'; i++)
	{
		USART_Transmit(a[i]);
	}
}
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



// onewire_search_state ow_search;

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
		
		_delay_ms(50);
		
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
					// create a 2 byte hash from the device's address - used for radio device id
					a1 = (ow_search_address[0] << 8 | ow_search_address[1]) ^
						(ow_search_address[2] << 8 | ow_search_address[3]) ^
						(ow_search_address[4] << 8 | ow_search_address[5]) ^
						(ow_search_address[6] << 8 | ow_search_address[7]);
					
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
					
					// wait before going to the next device - 16 seconds
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
