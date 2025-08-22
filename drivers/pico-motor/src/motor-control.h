#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                                   MACROS                                   */
/* -------------------------------------------------------------------------- */

#define TX_PACKET_SIZE  11
#define RX_PACKET_SIZE  8

#define START_BYTE 199
#define STOP_BYTE 200

/* -------------------------------------------------------------------------- */
/*                                   STRUCTS                                  */
/* -------------------------------------------------------------------------- */

/** 
 * @brief Struct that stores the received motor controller message
 */
typedef struct msg {
    uint8_t stering_angle;  /** < 0 to 180 degs */
    uint8_t speed;          /** < 0 to 180 degs */
    uint8_t ledColor;       /** < 0, 1, 2, 3 ,4 */
    bool ledBlinking;

    union {
        struct {
            uint8_t low;
            uint8_t high;
        } bytes;
        uint16_t word;
    } crc16;                /** < union that stores crc, `.word` (uint16) and `.bytes.low/high` (uint8) */

} msg_t;

/* -------------------------------------------------------------------------- */
/*                                    ENUMS                                   */
/* -------------------------------------------------------------------------- */

typedef enum msg_status {
    MSG_SUCCESS = 0,
    MSG_TIMEOUT = -1
} msg_status_t;

/* -------------------------------------------------------------------------- */
/*                                   EXTERNS                                  */
/* -------------------------------------------------------------------------- */

// // I am copying how the Arduino Motor Carrier did reading/write for consistency. 
// // Personally, I think this method is cursed...
// extern volatile bool messageStarted;
// extern volatile bool messageComplete;
// extern uint8_t receivedMessage[RX_PACKET_SIZE - 2];

/* -------------------------------------------------------------------------- */
/*                                 PROTOTYPES                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Function that reads UART and stores to the passed msg_t pointer
 * 
 * @param message Pointer to a message struct
 * @return Error Statues
 * @retval 0 - New message
 * @retval -1 - Timeout
 */
int read_motor_message(msg_t* message);

/**
 * @brief Calculates the msg's crc
 * 
 * @param message (const msg_t *) Pointer to the message 
 * @return `uint16_t` CRC value of passed message
 */
uint16_t calculate_msg_crc(const msg_t* message);

/**
 * @brief Parses message to make sure CRC is correct meaning the information in the message is correct
 * 
 * @param message (const msg_t *) Pointer to the received message buffer
 * @return `bool` weither or not the passed message is correct and can be uses
 */
bool parse_received_message(const msg_t* message);

/**
 * @brief Some kind of response message function, idk
 * 
 * @param Correct 
 */
void send_response(const bool Correct);

#endif // MOTOR_CONTROL_H