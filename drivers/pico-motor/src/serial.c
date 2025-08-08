#include "serial.h"

#include "pico/stdlib.h"


/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

void send_USB_UART(const uint8_t *data, size_t length) {
    for (int i = 0; i < length; i++) {
        putchar_raw(data[i]);
    }
}




