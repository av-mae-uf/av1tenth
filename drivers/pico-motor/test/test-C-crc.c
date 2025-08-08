#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint16_t initial;
    uint16_t polynome;
} crc16_config;

uint16_t crc16_ccitt(const uint8_t* data, size_t length, const crc16_config* cfg) {
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

        crc &= 0xFFFF;
    }

    return crc;
}

void test_crc(const char* label, const uint8_t* data, size_t len, uint16_t expected) {
    crc16_config config = {
        .initial = 0xFFFF,
        .polynome = 0x1021
    };

    uint16_t result = crc16_ccitt(data, len, &config);

    printf("%-20s | Length: %3zu | CRC: 0x%04X | %s\n", label, len, result, (result == expected ? "OK" : "FAIL"));
}

int main() {
    test_crc("Empty", (uint8_t*)"", 0, 0x1D0F);
    test_crc("A", (uint8_t*)"A", 1, 0x9479);
    test_crc("123456789", (uint8_t*)"123456789", 9, 0xE5CC);

    uint8_t long_data[256];
    memset(long_data, 'A', 256);
    test_crc("256 x 'A'", long_data, 256, 0xE938);

    return 0;
}
