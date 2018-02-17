# Wirekite

Wirkekite let's you wire up digital and analog inputs and outputs to your Mac or Windows computer. From there, you control them with software written in Objective-C, Swift or .NET running on your computer. 

To connect the inputs and outputs, use a [Teensy board](https://www.pjrc.com/teensy/) connected via USB. It looks a lot like an Arduino Nano connected for programming. Yet with Wirekite the custom code is written for and run on your computer – not for the microcontroller.

See the [Wiki](https://github.com/manuelbl/Wirekite/wiki) for more information and how to get started.


## Supported boards

- [Teensy LC](https://www.pjrc.com/store/teensylc.html)
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)


## Supported inputs / outputs / protocols

- Digital output
- Digital input
- Analog input
- PWM output
- I2C
- SPI


## Repositories

There are three repositories:

 - [Wirekite](https://github.com/manuelbl/Wirekite) – code for the Teensy board and home of the [Wiki](https://github.com/manuelbl/Wirekite/wiki) (this repository)
 - [WirekiteMac](https://github.com/manuelbl/WirekiteMac) – the Mac libraries for using the Wirekite in Objective-C or Swift on a Macintosh
 - [WirekiteWin](https://github.com/manuelbl/WirekiteWin) – the .NET libraries for using the Wirekite in C# or VB.NET on a Windows computer


## Building the Teensy software yourself (rarely needed)

There is no need to build the Teensy software yourself as
you can simply [install a prebuilt binary](https://github.com/manuelbl/Wirekite/wiki/Prepare-the-Teensy-Board).
If still want to build it yourself, stick to this [guide](https://github.com/manuelbl/Wirekite/wiki/Build-Wirekite-Microcontroller-Code-Yourself).
