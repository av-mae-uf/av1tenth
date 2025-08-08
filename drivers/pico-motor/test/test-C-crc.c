#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief Struct for CRC16, not sure if it is flexible, but ¯\_(ツ)_/¯
 */
typedef struct crc16_cfg {
  uint16_t initial;         /**< Initial CRC value (e.g., 0x0000 for XMODEM) */
  uint16_t polynome;        /**< CRC polynomial (e.g., 0x1021 for CRC-CCITT) */
  uint16_t final_xor_value; /**< Final XOR value of CRC16, not implemented yet  */
  bool reverse_input;       /**< Reverses inputed value before starting algorithm if True, not implemented yet */
  bool reverse_output;      /**< Reverses Output after algorithm is done, not implemented yet */
} crc16_config;

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

void test_crc(const char* label, const uint8_t* data, size_t len, uint16_t expected) {
    crc16_config config = {
        .initial = 0x0000,
        .polynome = 0x1021
    };

    uint16_t result = crc16_xmodem(data, len, &config);
    // uint16_t result = crc16_xmodem(data, len, NULL);

    printf("%-20s | Length: %3zu | CRC: 0x%04X | %s\n", label, len, result, (result == expected ? "OK" : "FAIL"));
}

int main() {
    printf("Testing CRC 16 XModem config\n");
    test_crc("Empty", (uint8_t*)"", 0, 0x0000);
    test_crc("A", (uint8_t*)"A", 1, 0x58E5);
    test_crc("123456789", (uint8_t*)"123456789", 9, 0x31C3);

    uint8_t long_data[256];
    memset(long_data, 'A', 256);
    test_crc("256 x 'A'", long_data, 256, 0xABE3);

    return 0;
}
