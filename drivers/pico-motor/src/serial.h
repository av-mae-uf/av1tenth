#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/*                                   MACROS                                   */
/* -------------------------------------------------------------------------- */

#define BAUDRATE        115200

/* -------------------------------------------------------------------------- */
/*                                 PROTOTYPES                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send a USB message with data and length
 * 
 * @param data Pointer to the data buffer
 * @param len  Length of the data buffer
 */
void send_USB_UART(const uint8_t *data, size_t length);

#endif // SERIAL_H