# Pico Motor

This directory contains all files related to the Pico motor project.  
The Arduino motor carrier’s PWM has a limitation where it can only set the motor speed to two distinct values. This occurs because the Arduino’s PWM is 8-bit (0–255), while servos only use a small portion of that range (typically 7–28).  
In our case, we use the range, 14–21, where 17 represents the midpoint.


## src Directory

This directory stores the source code for the microcontroller.

- **`main.c`** – Initializes all components and runs the functions implemented in the helper files below.  
- **`crc16.h/.c`** – Implements CRC16 checksum functionality.  
- **`serial.h/.c`** – Contains UART code for communication between the microcontroller and the computer.  
- **`servo-pwm.h/.c`** – Provides generic PWM control code.  
- **`motor-control.h/.c`** – Contains application-specific motor control code that integrates the helper modules above.


## build Directory

This directory stores the compiled file `pico-motor.uf2`, which is required to upload the C code to the Pico.  
Use this directory to build the latest version of the firmware.

To build the Pico code, you must first install the Pico C SDK and build tools.

Once the toolkit is installed, cd to the `build` directory and run:

```bash
cmake ..
make
```

## PCB Directory

This directory contains the KiCad project files for the Pico Motor breakout board.

If you need to order more boards, use the production-ready ZIP file located at: `PCB/production/pico-motor-breakout.zip`

You can upload this file directly to JLCPCB to manufacture new boards.

