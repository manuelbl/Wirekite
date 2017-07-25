/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//
// Extract from analog.c
//

// the alternate clock is connected to OSCERCLK (16 MHz).
// datasheet says ADC clock should be 2 to 12 MHz for 16 bit mode
// datasheet says ADC clock should be 1 to 18 MHz for 8-12 bit mode

#if F_BUS == 120000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1) // 7.5 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 15 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 15 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 15 MHz
#elif F_BUS == 108000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1) // 7 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 14 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 14 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 14 MHz
#elif F_BUS == 96000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 24 MHz
#elif F_BUS == 90000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 11.25 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 11.25 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 11.25 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 22.5 MHz
#elif F_BUS == 80000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 20 MHz			
#elif F_BUS == 72000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 9 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 18 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 18 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 18 MHz
#elif F_BUS == 64000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 8 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 16 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 16 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 16 MHz
#elif F_BUS == 60000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 7.5 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 15 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 15 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 15 MHz
#elif F_BUS == 56000000 || F_BUS == 54000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1) // 7 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 14 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 14 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 14 MHz
#elif F_BUS == 48000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 12 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1) // 24 MHz
#elif F_BUS == 40000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 10 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1) // 20 MHz
#elif F_BUS == 36000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1) // 9 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1) // 18 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1) // 18 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1) // 18 MHz
#elif F_BUS == 24000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0) // 12 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0) // 12 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0) // 12 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 24 MHz
#elif F_BUS == 16000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 16 MHz
#elif F_BUS == 8000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 8 MHz
#elif F_BUS == 4000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 4 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 4 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 4 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 4 MHz
#elif F_BUS == 2000000
  #define ADC_CFG1_16BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 2 MHz
  #define ADC_CFG1_12BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 2 MHz
  #define ADC_CFG1_10BIT  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 2 MHz
  #define ADC_CFG1_8BIT   ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0) // 2 MHz
#else
#error "F_BUS must be 120, 108, 96, 90, 80, 72, 64, 60, 56, 54, 48, 40, 36, 24, 4 or 2 MHz"
#endif
