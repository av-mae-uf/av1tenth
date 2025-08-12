import serial
from crc import Calculator, Crc16
import math

# ---------------------------------------------------------------------------- #
#                                  PARAMETERS                                  #
# ---------------------------------------------------------------------------- #

MAX_SPEED = 585 * (2 * math.pi * 60e-3) / 60

isSpeedLimited = False

PORT = '/dev/ttyACM0'
BAUD = 115200
TIMEOUT_S = 0.001

def convert_speed_to_angle(speed: float) -> int:
    '''
        Converts pass speed into a angle [0,180].\n 
        Uses Global MAX_SPEED to convert speed to angle\n
        Uses Global isSpeedLimited to to inhibit speed to a different range\n
    '''
    global MAX_SPEED
    global isSpeedLimited

    # this is the equation to convert speed to an angle for the Pico/Arduino done by Aditya and Patrick
    # I believe it uses the circumference of the wheel?
    angle = 90 + speed * (72 / MAX_SPEED)
    angle = int(angle)

    if isSpeedLimited:
        angle = min(angle, 110)
        angle = max(angle, 70)
    else:
        angle = min(angle,180)
        angle = max(angle,0)

    return angle

def get_msg(angle: int) -> bytearray:
    ''' 
        Converts angle to a message byte array to send to the serial device
    '''
    calc = Calculator(Crc16.XMODEM)
    data_bytes = bytearray([90, int(angle), 2, 0])
    crc_val = calc.checksum(data_bytes)
    msg = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc_val >> 8) & 0xFF, crc_val & 0xFF, 200 ])

    return msg

# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #

def main():
    try: 
        pico = serial.Serial(port=PORT, baudrate= BAUD, timeout=TIMEOUT_S)
        print("Connected to Pico!")
        
        while (True):
            speed = float(input("Type new speed (m/s) [-6, 6] or `37` to end the program: "))

            if speed == 37 :
                print("Exiting program")
                break

            while (speed < -6 or speed > 6):
                speed = float(input("Error, invalid input! Please type new speed [-6, 6] or `37` to end the program: "))

            speed_data = convert_speed_to_angle(speed)

            msg = get_msg(speed_data)
            print(f"Sending {speed} m/s | Servo Angle Equivalent = {speed_data} deg | HEX MSG: {' '.join(f'{byte:02X}' for byte in msg)}")
            pico.write(msg)

            response = pico.read_until()
            if len(response) == 3:
                if response == bytearray([0x89, 0x80, 0x7F]):
                    print(f"SUCCESS: Response = {' '.join(f'{byte:02X}' for byte in response)}")
                elif response == bytearray([0x89, 0x40, 0x7F]):
                    print(f"FAILURE: Response = {' '.join(f'{byte:02X}' for byte in response)}")
                else:
                    print(f"UNKNOWN RESPONSE: {' '.join(f'{byte:02X}' for byte in response)}")
            else:
                print(f"INVALID LENGTH: got {len(response)} bytes → {response}")

        pass
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt has occured... Exiting now!")
    # except Exception as e:
    #     print(f"Exception {e} has occured...")
    finally:
        pico.close()


if __name__ == "__main__":
    main()