#!/bin/sh
make clean
make teensylc MCU=MKL26Z64 CPU=cortex-m0plus F_CPU=48000000
make clean
make teensy32 MCU=MK20DX256 CPU=cortex-m4 F_CPU=72000000
