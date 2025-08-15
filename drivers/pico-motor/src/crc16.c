#include "crc16.h"

/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Function that reverses word passed
 * 
 * @param w (uint8_t)
 * @return `uint8_t`
 */
static uint16_t reverse_word(uint16_t w) {
    uint16_t r = 0x00;
    for (int i = 0; i < 16; i++) {
        r = (uint16_t)((r << 1) | (w & 1));
        w >>= 1;
    }
    return r;
}

uint16_t crc16(const uint8_t* data, size_t length, const crc16_config* cfg) {
    uint16_t crc = 0x0000;
    uint16_t polynome = 0x1021;
    uint16_t xorout = 0x0000;
    bool refin = false;
    bool refout = false;

    if (cfg != NULL) {
        crc = cfg->initial;
        polynome = cfg->polynome;
        xorout = cfg->final_xor_value;
        refin = cfg->reverse_input;
        refout = cfg->reverse_output;
    }

    if (!refin) {
        for (size_t i = 0; i < length; i++) {
            crc ^= ((uint16_t)data[i]) << 8;

            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ polynome;
                } else {
                    crc <<= 1;
                }
            }
        }
    } else {
        uint16_t reverse_poly = reverse_word(polynome);
        for (size_t i = 0; i < length; i++) {
            crc ^= ((uint16_t)data[i]);

            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (uint16_t) ((crc >> 1) ^ reverse_poly);
                } else {
                    crc >>= 1;
                }
            }
        }
    }
    if (refout && !refin) {
        crc = reverse_word(crc);
    }

    crc ^= xorout;
    return crc;
}
