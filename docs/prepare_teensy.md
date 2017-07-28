# Prepare the Teensy for Wirekite

There is no need to build the Teensy code yourself. It is the same code code for everybody. Your energy should go into the Windows or Mac program. So to prepare your Teensy, you just need to load the software once:

1. Download and install the Teensy Loader from [PJRC](https://www.pjrc.com):
    - MacOS: [Teensy Loader for MacOS](https://www.pjrc.com/teensy/teensy.dmg)
    - Windows: [Teensy Loader for Windows](https://www.pjrc.com/teensy/teensy.exe)
    - Linux: [Teensy Loader for Ubuntu Linux](https://www.pjrc.com/teensy/loader_linux.html)

2. Download the Teensy binary code (.hex file):

    - Teensy LC: [wirekite_teensylc.hex](https://raw.githubusercontent.com/manuelbl/Wirekite/master/bin/wirekite_teensylc.hex) (right-click and download; otherwise it opens in the browser as it is an ASCII file)

3. Connect your Teensy to your Mac / Windows / Linux computer

4. Start the Teensy Loader and open downloaded .hex file (from *File* menu or top left icon in the toolbar)

5. Enable the *automatic* mode (from the *Operations* menu or the top right icon in the toolbar)

6. Press the reset button on the Teensy board.

The Teensy Loader will then load the software into the flash memory of the board and reboot it. It only takes about 1 second.

You can always go back to use your Teensy for other purposes, such a programming it yourself with the Arduino IDE and Teensyduino. The original bootloader is not changed.

There is no need to install any drivers for the Wirekite. It is automatically recognized as a custom USB device by the Mac and Windows (Vista and later).
