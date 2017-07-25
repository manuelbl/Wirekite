# Wirekite

Wirkekite let's you wire up digital and analog inputs and outputs to your Mac or Windows computer. From there, you control them with software written in Objective-C, Swift or .NET running on your computer. 

To connect the inputs and outputs, use a [Teensy development board](https://www.pjrc.com/teensy/) connected via USB. It looks a lot like an Arduino Nano connected for programming but with Wirekite the custom code is written for and run on your computer – not for the microcontroller.

This repository contains the Teensy code. Macintosh and Windows libraries are coming soon to a repository nearby.

## Supported boards

- [Teensy LC](https://www.pjrc.com/store/teensylc.html)

## Supported inputs / outputs / protocols

- Digital output
- Digital input
- Analog input
- PWM output


## Prepare your Teensy

There is no need to build the Teensy code yourself. It is the same code code for everybody. Your energy should go into the Windows or Mac program. So to prepare your Teensy, you just need to load the software once:

1. Download and install the Teensy Loader from [PJRC](https://www.pjrc.com):
    - MacOS: [Teensy Loader for MacOS](https://www.pjrc.com/teensy/teensy.dmg)
    - Windows: [Teensy Loader for Windows](https://www.pjrc.com/teensy/teensy.exe)
    - Linux: [Teensy Loader for Ubuntu Linux](https://www.pjrc.com/teensy/loader_linux.html)

2. Download the Teensy binary code (.hex file):

    - Teensy LC: [https://raw.githubusercontent.com/manuelbl/Wirekite/master/bin/wirekite_teensylc.hex](https://raw.githubusercontent.com/manuelbl/Wirekite/master/bin/wirekite_teensylc.hex) (right-click and download; otherwise it opens in the browser as it is an ASCII file)

3. Connect your Teensy to your Mac / Windows / Linux computer

4. Start the Teensy Loader and open downloaded .hex file (from *File* menu or top left icon in the toolbar)

5. Enable the *automatic* mode (from the *Operations* menu or the top right icon in the toolbar)

6. Press the reset button on the Teensy board.

The Teensy Loader will then load the software into the flash memory of the board and reboot it. It only takes about 1 second.

You can always go back to use your Teensy for other purposes, such a programming it yourself with the Arduino IDE and Teensyduino. The original bootloader is not changed.

There is no need to install any drivers for the Wirekite. It is automatically recognized as a custom USB device by the Mac and Windows (Vista and later).


## Building the Teensy software yourself

If you want to build the software yourself - even though this is not necessary - it is quite straightforward.

1. Download the GNU ARM Embedded Toolchain (Teensy uses ARM processors)from [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm) and unpack it on your computer. I put it in a folder called *Software* within my *Documents* folder and renamed the unpacked folder *gcc-arm-none-eabi-6-2017-...* to *gcc-arm-none-eabi*.

2. Verify you have *GNU Make* installed on your computer. If not, then Windows users can download it from [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm) and Macintosh users should somehow fix their setup as GNU Make is part of every original MacOS installation.

```
$ make -v
GNU Make 3.81
...
```

3. Modify the *makefile* to match your Teensy board and environment, i.e. set the variables *MCU*, *CPU*, *F_CPU* and *TOOLDIR* at the top of the file. Read the comments for further instructions.

4. Build it

```
$ make
arm-none-eabi-gcc -Wall -mthumb -Os -MMD -D__MKL26Z64__ -DF_CPU=48000000 -Iinclude -mcpu=cortex-m0plus -fsingle-precision-constant -fno-common -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -fno-common -c src/main.cpp -o obj/main.o
...
arm-none-eabi-gcc -Wl,--gc-sections --specs=nano.specs -mcpu=cortex-m0plus -mthumb -Tsrc/MKL26Z64.ld obj/main.o obj/analog.o obj/buffers.o obj/digital_pin.o obj/kinetis.o obj/pwm.o obj/timer.o obj/uart0.o obj/usb.o obj/util.o obj/wirekite.o obj/yield.o   -lm -o bin/wirekite.elf
arm-none-eabi-objcopy -R .stack -O ihex bin/wirekite.elf bin/wirekite.hex
arm-none-eabi-size bin/wirekite.elf
   text    data     bss     dec     hex filename
   8508     404    2168   11080    2b48 bin/wirekite.elf
```
That's it. The resulting file is `bin/wirekite.hex` and can be loaded with the Teensy loader.

Use the IDE of your choice. I've used [Visual Studio Code](https://code.visualstudio.com/) from Microsoft – the new and slick IDE from Microsoft. You have to install the *C/C++* extension (click the *Extensions* icon on the left, search for *cpptools* and install *C/C++*).

### Bare metal

Wirekite is a bare-metal implementation, i.e. it does not depend on any operating system or major library such as [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html). However, the code uses some of Paul Stoffregen's great work and uses a few Teensyduino files without hardly any change. For more information, see [Integration of third-party code](https://github.com/manuelbl/Wirekite/blob/master/docs/integrate_tp_source.md).


## Coming soon

- Objective-C library for MacOS
- Library for Windows
- Support for Teensy 3.2
- I2C
