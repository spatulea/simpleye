# March 13, 2022 - A slow blink
It builds! It blinks! But it doesn't talk. Yet. Ok, the good news first:
## Building for the nano 33
The nRF5 SDK is somewhat "board focused" in its BSP implementation. I guess the expectation is that development is initially taking place using one of their evaluation boards. Long story short, the makefile now includes a board definition: 
```
CFLAGS += -DCUSTOM_BOARD_INC=board
```
So that `boards.h` can correctly parse and include `board.h`, the "BSP" header for this specific board:
```C
#elif defined (CUSTOM_BOARD_INC)
  #include STRINGIFY(CUSTOM_BOARD_INC.h)
```
To be fair, it would be simpler to just edit the `boards.h` file directly and get rid of the unused compiler conditionals, but that would against the spirit of a SDK or library. I may change my mind in the future and pull relevant parts of the SDK into this codebase, but for now I would rather treat it like an included library. On one hand, it would simplify the makefile and improve the accuracy of VSCode's intellisense by removing unused files. On the other, it's nice to separate the application code from Nordic's SDK and not "adopt" the SDK source. We'll see how the project progresses and re-evaluate as it makes sense.
### Flashing
One detail to note is that I decided to keep Arduino's bootloader in the flash memory for now in case it's useful later. With 1MB of NVM I'm not worried about running out of space. The linker script `simpleye_gcc_nrf52.ld` had to be modified to skip over the first 0x10000 addresses:
```C
/* Note, flash origin changed to 0x10000 to avoid overwriting the bootloader */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x10000, LENGTH = 0x100000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x40000
}
```
## Debug serial output
The easiest way to get a serial console with ARM's SWD interface is typically by using the SWO output from the MCU. Most debuggers conveniently include a UART input for it and voila! Easy printf console debugging! Not so with the nano 33... There is no exposed SWO pad with the other SWD signals (makes sense- SWO is not needed for programming during manufacturing) and the MCU's SWO pin was repurposed to... to be a pull-up voltage for I2C resistors?

<img src="assets/swo_pullup1.jpg" height=80/> <img src="assets/swo_pullup2.jpg" height=80/>
So that's interesting... not quite sure why the I2C resistors need a programmable pull-up, or why of available GPIOs it had to be SWO pin. That leave us with some alternative options:

1. If I2C1 is not used, then wire up the debugger's SWO input to the two resistor's "top side"
2. Repurpose another GPIO that is brought out to a board connector pin, solder the debugger to that pin and direct debug prints to that GPIO/UART
3. Configure the USB connection as a TTY device and send debug output that way

Option 3 is definitely tempting and elegant (no soldering), although I fear that resetting the MCU (like during programming) will disconnect the TTY device and require reconnecting every time, which gets annoying.


# March 11, 2022 - A slow start
The project is off to a slow start. Using an Arduino board as a "dev" board while bypassing the Arduino framework is not as straightforward as I'd initially thought. Long story below, but the TLDR is: **the nano 33 BLE needs an external debugger and soldering to its SWD programming interface in order to load and debug custom firmware.**

## The problem
Modern development and evaluation boards often interface with a PC over USB by integrating an on-board debugger. For example, the [micro:bit](https://microbit.org) implements the following [on-board debugger](https://tech.microbit.org/software/daplink-interface):
<img src= "https://tech.microbit.org/docs/software/assets/v2-interface.png" style="background-color:white;border:10px solid white;" />
The micro:bit *INTERFACE MCU*, a Freescale Cortex-M0+, implements a variety of convenient protocols over USB for interfacing with the PC, while communicating with the *TARGET MCU* (the MCU of interest when developing) over SWD (Serial Wire Debug) and sometimes UART (the micro:bit's addition of I2C is unusual). ST similarily often includes their ST-LINK/V2 debugger/programmer on their [evaluation boards](https://www.st.com/en/evaluation-tools/32f411ediscovery.html).

This is all a somewhat poor justification for my assumption that the Arduino nano 33 BLE did something similar. It does not. There is no in-built programmer or debugger on the nano 33 board. For this board at least (unsure about other non-Cortex Arduinos) Arduino uses a bootloader to accept and apply new firmware over USB directly on the main/target MCU. There is no separate interface MCU. Once a new arduino sketch is loaded in flash, the bootloader reboots and an *Arduino OS* ([mbed OS with Arduino-specific code](https://github.com/arduino/ArduinoCore-mbed)) takes over, implements the USB serial interface and runs the user's sketch (I think). Anyway, before I go too far down the Arduino-mbed-RTOS rabbit hole, the end result is that I can't just load arbitrary firmware to the nRF52840 over the built-in USB interface. And I don't really want to deal with the bootloader and/or mbed-os. Not for this project.

## SWD to the rescue
Thankfully, Arduino had to first program the bootloader/os onto the boards before shipping, and that means there's an exposed SWD (single-wire debug) interface! [The internet](https://hackaday.io/project/168903/instructions) was quick to find said port and provide some.... information, almost a guide, for wiring it up.

![image](https://cdn.hackaday.io/images/5180881576180008097.jpg)

The 5 SMT pads highlighted on the left are the SWD interface in the following clockwise order, starting with the top-left pad:
- Reset
- Ground
- SWCLK
- Vdd
- SWDIO

## Alternatives
For those who want to avoid soldering a separate debugger, there may be other options acceptable for your project:
### Zephyr
The [Zephyr project](https://zephyrproject.org) supports the nano 33 BLE sense and includes [documentation](https://docs.zephyrproject.org/latest/boards/arm/arduino_nano_33_ble/doc/index.html) for flashing firmware using the bootloader and USB connection. So one could either target the nano 33 with a Zephyr-based project or dig into Zephyr's source and find the [flash address range](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/arm/arduino_nano_33_ble/arduino_nano_33_ble.dts) of the bootloader.
### Arduino
Technically, all of the functional goals of this project can be accomplished entirely with the Arduino framework. There's a good [Applications of TinyML](https://www.edx.org/course/applications-of-tinyml) course on edX.org that implements various machine learning applications, including machine vision, using Arduino and Tensorflow Lite/Micro.