#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief Struct for CRC16, not sure if it is flexible, but ¯\_(ツ)_/¯
 */
typedef struct crc16_config {
  uint16_t initial;         /**< Initial CRC value (e.g., 0x0000 for XMODEM) */
  uint16_t polynome;        /**< CRC polynomial (e.g., 0x1021 for CRC-CCITT) */
  uint16_t final_xor_value; /**< Final XOR value of CRC16, not implemented yet  */
  bool reverse_input;       /**< Reverses inputed value before starting algorithm if True, not implemented yet */
  bool reverse_output;      /**< Reverses Output after algorithm is done, not implemented yet */
} crc16_config;

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


/**
 * @brief Computes CRC-16 using XMODEM Algorithm (MSB-First).
 * 
 * If `cfg` is NULL, it defaults to CRC-16-XMODEM (poly 0x1021, init 0xFFFF).
 * 
 * @param data (const uint8_t *) Pointer to the input data buffer
 * @param length (size_t) Number of bytes in the input data buffer.
 * @param cfg   (const crc_config *) Pointer to CRC config struct (nullable).
 * @return `[uint16_t]` : Computed 16 bit CRC value.
 */
uint16_t crc16_xmodem(const uint8_t* data, size_t length, const crc16_config* cfg) {
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

    // printf("Initial = %04X, Poly = %04X, Final XOR = %04X, RIN = %d, ROUT = %d\n", crc, polynome, xorout, refin, refout);


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

void test_crc(const char* label, const uint8_t* data, size_t len, uint16_t expected, crc16_config *cfg) {
    uint16_t result;
    if (cfg != NULL) {
        result = crc16_xmodem(data, len, cfg);
    } else {
        result = crc16_xmodem(data, len, NULL);
    }

    // uint16_t result = crc16_xmodem(data, len, NULL);

    printf("%-20s | Length: %3zu | CRC: 0x%04X | %s\n", label, len, result, (result == expected ? "OK" : "FAIL"));
}

int main() {
    printf("Testing CRC 16 XModem config\n");
    test_crc("Empty", (uint8_t*)"", 0, 0x0000, NULL);
    test_crc("A", (uint8_t*)"A", 1, 0x58E5, NULL);
    test_crc("123456789", (uint8_t*)"123456789", 9, 0x31C3, NULL);

    uint8_t long_data[256];
    memset(long_data, 'A', 256);
    test_crc("256 x 'A'", long_data, 256, 0xABE3, NULL);

    crc16_config MAXIM = {
        .initial = 0x0000,
        .polynome = 0x8005,
        .final_xor_value = 0xFFFF,
        .reverse_input = true,
        .reverse_output = true
    };

    printf("Testing CRC 16 MAXIM config\n");
    test_crc("Empty", (uint8_t*)"", 0, 0xFFFF, &MAXIM);
    test_crc("A", (uint8_t*)"A", 1, 0xCF3F, &MAXIM);
    test_crc("123456789", (uint8_t*)"123456789", 9, 0x44C2, &MAXIM);
    test_crc("256 x 'A'", long_data, 256, 0xAAA6, &MAXIM);

    crc16_config USB = {
        .initial = 0xFFFF,
        .polynome = 0x8005,
        .final_xor_value = 0xFFFF,
        .reverse_input = true,
        .reverse_output = true
    };

    printf("Testing CRC 16 USB config\n");
    test_crc("Empty", (uint8_t*)"", 0, 0x0000, &USB);
    test_crc("A", (uint8_t*)"A", 1, 0x8F80, &USB);
    test_crc("123456789", (uint8_t*)"123456789", 9, 0xB4C8, &USB);
    test_crc("256 x 'A'", long_data, 256, 0xCE19, &USB);

    return 0;
}
