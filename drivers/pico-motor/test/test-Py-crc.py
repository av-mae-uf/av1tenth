import serial
from crc import Calculator, Crc16, Configuration



def main(): 
    try:
        # pico = serial.Serial(port="/dev/ttyACM0", baudrate=115200)

        # apparently CRC version 7+ removed CRC CCITT, WHY????? (found explanation: https://github.com/Nicoretti/crc/issues/148)
        # linke to settings: https://srecord.sourceforge.net/crc16-ccitt.html
        config = Configuration (
            width=16,
            polynomial=0x1021,
            init_value=0x0000,
            final_xor_value=0x0000,
            reverse_input=False,
            reverse_output=False
        )
        
        calc = Calculator(config)
       
        # After looking at the arduino code, I think it is using CRC 16 XModem instead of CCITT...

        calc2 = Calculator(Crc16.XMODEM)

        buf = bytearray([90, 90, 2, 0])
        crc_val = calc.checksum(buf)
        print(crc_val)

        print("My config of XModem")
        print(f"Empty:       {hex(calc.checksum(b''))} (Expected: 0x0000)")
        print(f'A:           {hex(calc.checksum(b"A"))} (Expected: 0x58E5)')
        print(f"123456789:   {hex(calc.checksum(b'123456789'))} (Expected: 0x31C3)")
        print(f"256 x 'A':   {hex(calc.checksum(b'A' * 256))} (Expected: 0xABE3)")

        print("\nPremade Config of XModem")
        print(f"Empty:       {hex(calc2.checksum(b''))} (Expected: 0x0000)")
        print(f'A:           {hex(calc2.checksum(b"A"))} (Expected: 0x58E5)')
        print(f"123456789:   {hex(calc2.checksum(b'123456789'))} (Expected: 0x31C3)")
        print(f"256 x 'A':   {hex(calc2.checksum(b'A' * 256))} (Expected: 0xABE3)")

        pass
    except Exception as e:
        print(f"Exception {e} has occured ending program now!")

if __name__ == "__main__":
    main()
    