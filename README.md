# Wirekite

Wirkekite let's you wire up digital and analog inputs and outputs to your Mac or Windows computer. From there, you control them with software written in Objective-C, Swift or .NET running on your computer. 

To connect the inputs and outputs, use a [Teensy board](https://www.pjrc.com/teensy/) connected via USB. It looks a lot like an Arduino Nano connected for programming. Yet with Wirekite the custom code is written for and run on your computer – not for the microcontroller.

Once you have prepared the Teensy board ([one-time setup](docs/prepare_teensy.md)), you should be using the [MacOS code](https://github.com/manuelbl/WirekiteMac) or the Windows code (coming soon). Only few people will work with this repository.


## Supported boards

- [Teensy LC](https://www.pjrc.com/store/teensylc.html)
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) (soon)


## Supported inputs / outputs / protocols

- Digital output
- Digital input
- Analog input
- PWM output
- I2C (soon)



## Building the Teensy software yourself (rarely needed)

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

- Library for Windows
- Support for Teensy 3.2
- I2C
