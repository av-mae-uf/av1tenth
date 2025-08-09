#include "motor-control.h"
// #include "serial.h"
#include "crc16.h"

#include "pico/stdlib.h"

/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

int read_motor_message(msg_t* message) {
    static int counter = 0;

    bool messageStarted = false;
    bool messageComplete = false;
    uint8_t receivedMessage[RX_PACKET_SIZE];
    
    // TODO: see if there is a more effiencent way without the two bools
    // this is the original algorithm from the Arduino Motor Carrier
    int ch  = getchar_timeout_us(0);

    if (ch == PICO_ERROR_TIMEOUT) {
        return -1;
    }

    while (ch != PICO_ERROR_TIMEOUT) {
        
        uint8_t inByte = (uint8_t) ch;
        // putchar_raw(inByte);

        if (messageStarted) {
            if (messageComplete) {
                messageComplete = false;
                counter = 0;
            }   
            
            if ((inByte == STOP_BYTE) && (counter == RX_PACKET_SIZE - 2)) {
                messageComplete = true;
                messageStarted = false;
                counter = 0;
            } else if (counter < RX_PACKET_SIZE - 2) {
                receivedMessage[counter++] = inByte;
            }
            
        } else if (inByte == START_BYTE) {
            messageStarted = true;
            // counter = 0;
        }

        ch = getchar_timeout_us(0); 
    }

    // TODO make this better if possible
    message->stering_angle = receivedMessage[0];
    message->speed = receivedMessage[1];
    message->ledColor = receivedMessage[2];
    message->ledBlinking = receivedMessage[3];

    // TODO: check is this is the right
    message->crc16.bytes.high = receivedMessage[4];
    message->crc16.bytes.low = receivedMessage[5];


    return 0;
}


uint16_t calculate_msg_crc(const msg_t* message) {
    uint8_t buf[RX_PACKET_SIZE-4] = {message->stering_angle, message->speed, message->ledColor, message->ledBlinking};
    return crc16_xmodem(buf, RX_PACKET_SIZE-4, NULL);
}


bool parse_received_message(const msg_t* message) {
    
    if (message == NULL)
        return false;

    uint16_t crc_val = calculate_msg_crc(message);
    if (crc_val == message->crc16.word) {
        return true;
    }

    return false;
}

void send_response(const bool Correct) {
    putchar_raw(137);
    
    if(Correct) {
        putchar_raw(128);
    } else {
        putchar_raw(64);
    }

    putchar_raw(127);
}

/* -------------------------------------------------------------------------- */
/*                                   IGNORE                                   */
/* -------------------------------------------------------------------------- */

// this function is cursed, i am only writing it incase I might need it in the future....
// void sendMessage() {
//     uint8_t sendMessageData[TX_PACKET_SIZE];
//     uint8_t stateValue = 0;
//     uint16_t rpm_uint16 = 0;
//     uint16_t usigned_heading = 0;

    // // IDK what these were for
    // typedef enum LED_STATE {
    //     ACTIVE = 1,
    //     INACTIVE = 2
    // } LED_STATE_t;
//     LED_STATE_t State = ACTIVE;
//     switch (expression)
//     {
//         case Active:
//             stateValue = 64;
//             break;
//         case INACTIVE:
//             stateValue = 32;
//             break;
//     }

//     sendMessageData[0] = 157;
//     sendMessageData[1] = stateValue;

//     // rpm_uint16 = (int16_t)(RPM1 * 10.0);
//     sendMessageData[2] = (rpm_uint16 >> 8) & 0xFF;
//     sendMessageData[3] = rpm_uint16 & 0xFF;

//     // rpm_uint16 = (int16_t)(RPM2 * 10.0);
//     sendMessageData[4] = (rpm_uint16 >> 8) & 0xFF;
//     sendMessageData[5] = rpm_uint16 & 0xFF;

//     // TODO calculate enu_heading
//     sendMessageData[6] = (enu_heading>>8)&0xFF;
//     sendMessageData[7] = enu_heading&0xFF;

//     // TODO Calculate CRC16
//     sendMessageData[8] = (crc16>>8)&0xFF; // High bits
//     sendMessageData[9] = crc16&0xFF; // Low bits

//     sendMessageData[10] = 147;

//     send_USB_UART(sendMessageData, TX_PACKET_SIZE);
// }