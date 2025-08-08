#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/*                                   STRUCTS                                  */
/* -------------------------------------------------------------------------- */

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

/* -------------------------------------------------------------------------- */
/*                                 PROTOTYPES                                 */
/* -------------------------------------------------------------------------- */

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
uint16_t crc16_xmodem(const uint8_t* data, size_t length, const crc16_config* cfg);

#endif // CRC16_H