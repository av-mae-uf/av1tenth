import serial
from crc import Calculator, Crc16, Configuration



def main(): 
    try:
        # pico = serial.Serial(port="/dev/ttyACM0", baudrate=115200)

        # apparently CRC version 7+ removed CRC CCITT, WHY?????
        # linke to settings: https://srecord.sourceforge.net/crc16-ccitt.html
        config = Configuration (
            width=16,
            polynomial=0x1021,
            init_value=0xFFFF,
            final_xor_value=0x0000,
            reverse_input=False,
            reverse_output=False
        )

        calc = Calculator(config)

        buf = bytearray([90, 90, 2, 0])
        crc_val = calc.checksum(buf)
        print(crc_val)


        # TEST CASES TO SEE IF CONFIG IS GOOD
        # inputs None -> 0x1D0F | len 0 | BAD 0xFFFF
        # A -> 0x9479 | len 1  | BAD 0xB915
        # 123456789 -> 0xE5CC | len 9 | BAD 0x29B1
        #A string of 256 upper case “A” characters with no line breaks -> 0xE938 | len 256 | BAD 0xEA0B
        print(f"Empty:       {hex(calc.checksum(b''))} (Expected: 0x1D0F)")
        print(f'A:           {hex(calc.checksum(b"A"))} (Expected: 0x9479)')
        print(f"123456789:   {hex(calc.checksum(b'123456789'))} (Expected: 0xE5CC)")
        print(f"256 x 'A':   {hex(calc.checksum(b'A' * 256))} (Expected: 0xE938)")


        pass
    except Exception as e:
        print(f"Exception {e} has occured ending program now!")

if __name__ == "__main__":
    main()
    