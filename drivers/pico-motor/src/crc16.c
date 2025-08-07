#include "crc16.h"

/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

uint16_t crc16_ccitt(const uint8_t* data, size_t length, const crc16_config* cfg) {
    // TODO: Needs to be tested...

    uint16_t crc = 0xFFFF;
    uint16_t polynome = 0x1021;

    if (cfg != NULL) {
        crc = cfg->initial;
        polynome = cfg->polynome;
    }

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

    return crc;
}
