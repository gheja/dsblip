#include <stdint.h>
#include <util/crc16.h>

#include "defines.h"
#include "macros.h"

#include "utils.h"

void gset_output_high() {
    BIT_SET(OW_PORT, OW_BIT);
}

void gset_output_low() {
    BIT_CLEAR(OW_PORT, OW_BIT);
}

void gset_input_pullup() {
    OW_DDR &= ~_BV(OW_BIT);
    gset_output_high();
}

void gset_input_hiz() {
    BIT_CLEAR(OW_DDR, OW_BIT);
    gset_output_low();
}

void gset_output() {
    OW_DDR |= _BV(OW_BIT);
}

void gset_bit() {
    OW_PORT |= _BV(OW_BIT);
}

void gclear_bit() {
    OW_PORT &= ~_BV(OW_BIT);
}

uint8_t gread_bit() {
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
