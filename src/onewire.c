#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>

#include "onewire.h"

#include "utils.h"

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

uint8_t onewire_get_first_address_byte()
{
    return ow_search_address[0];
}

// create a 2 byte hash from the device's address - used for radio device id
uint16_t onewire_get_address_hash()
{
    return
        (ow_search_address[0] << 8 | ow_search_address[1]) ^
        (ow_search_address[2] << 8 | ow_search_address[3]) ^
        (ow_search_address[4] << 8 | ow_search_address[5]) ^
        (ow_search_address[6] << 8 | ow_search_address[7]);
}
