# Build Wirekite Yourself

You should rarely need to built the Wirekite software for the Teensy board yourself
as you can install a pre-built binary on your Teensy. Your energy should usually go
into writing software for your Mac or Windows computer using the
[Wirekite MacOS library](https://github.com/manuelbl/WirekiteMac)
or the Wirekite Windows library (coming soon).

If you want to build it yourself, it is quite straightforward.

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
