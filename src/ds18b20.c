#include <stdint.h>
#include <stdbool.h>

#include "ds18b20.h"

#include "onewire.h"
#include "utils.h"

static const int8_t kScratchPadLength = 9;

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
	if (onewire_get_first_address_byte() == 0x10)
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
