# Wirekite

Wirkekite let's you wire up digital and analog inputs and outputs to your Mac or Windows computer. From there, you control them with software written in Objective-C, Swift or .NET running on your computer. 

To connect the inputs and outputs, use a [Teensy board](https://www.pjrc.com/teensy/)connected via USB. It looks a lot like an Arduino Nano connected for programming. Yet with Wirekite the custom code is written for and run on your computer – not for the microcontroller.

Once you have prepared the Teensy board ([one-time setup](docs/prepare_teensy.md)), you should be using the [MacOS code](https://github.com/manuelbl/WirekiteMac) or the [Windows code](https://github.com/manuelbl/WirekiteWin). Only few people will work with this repository.


## Supported boards

- [Teensy LC](https://www.pjrc.com/store/teensylc.html)
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) (soon)


## Supported inputs / outputs / protocols

- Digital output
- Digital input
- Analog input
- PWM output
- I2C


## Building the Teensy software yourself (rarely needed)

There is no need to build the Teensy software yourself as
you can simply [install a prebuilt binary](docs/prepare_teensy.md).
If still want to build it yourself, stick to this [guide](docs/build_wirekite.md).

## Coming soon

- Support for Teensy 3.2
