# Integration of Third-party code

## Teensy / Teensyduino

Some code originates from Teensyduino's core libraries (https://github.com/PaulStoffregen/cores)

To integrate it:

- Save teensy3/kinetis.h as include/kinetis.h
- Save teensy3/mk20dx128.c as src/kinetis.c
- Save teensy3/mkl26z64.ld as src/MKL26Z64.ld
- Save teensy3/mk20dx256.ld as src/MK20DX256.ld

Modify kinetis.c:

- Uncomment line `#include "core_pins.h" // testing only` (approx. line 32)
- Uncomment line `#include "ser_print.h" // testing only` (approx. line 33)
- Uncomment line `void _init_Teensyduino_internal_(void) __attribute__((noinline));` (approx. line 67)
- Uncomment line `_init_Teensyduino_internal_();` (approx. line 1095)
