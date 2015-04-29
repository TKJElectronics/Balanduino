/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $

  Changelog
  -----------
  11/25/11  - ryan@ryanmsutton.com - Add pins for Sanguino 644P and 1284P
  07/15/12  - ryan@ryanmsutton.com - Updated for arduino0101
  01/26/13  - kristianl@tkjelectronics.com - Changed to work with Balanduino
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

static const uint8_t SS   = 6;
static const uint8_t MOSI = 27;
static const uint8_t MISO = 28;
static const uint8_t SCK  = 29;

#define LED_BUILTIN 6

static const uint8_t A0 = 7;
static const uint8_t A1 = 8;
static const uint8_t A2 = 9;
static const uint8_t A3 = 10;
static const uint8_t A4 = 11;
static const uint8_t A5 = 12; // Not broken out - used for battery voltage measurement

static const uint8_t SDA = 13;
static const uint8_t SCL = 14;

// ATMEL ATMEGA1284P / Balanduino revision 1.2 or older
//
//                   +---\/---+
//   M2A (D25) PB0  1|        |40 PA0 (A0 / D7)
//   M2B (D26) PB1  2|        |39 PA1 (A1 / D8)
//   INT2 (D2) PB2  3|        |38 PA2 (A2 / D9)
// BUZZER (D5) PB3  4|        |37 PA3 (A3 / D10)
//    LED (D6) PB4  5|        |36 PA4 (A4 / D11)
//  MOSI (D27) PB5  6|        |35 PA5 (A5 / D12) VBAT
//  MISO (D28) PB6  7|        |34 PA6 (D30) EN1B
//   SCK (D29) PB7  8|        |33 PA7 (D31) EN2B
//             RST  9|        |32 AREF
//             VCC 10|        |31 GND
//             GND 11|        |30 AVCC
//           XTAL2 12|        |29 PC7 (D24) M1B
//           XTAL1 13|        |28 PC6 (D23) M1A
//     RX (D0) PD0 14|        |27 PC5 (D22) M2EN
//     TX (D1) PD1 15|        |26 PC4 (D21) M1EN
//  EN1A (D15) PD2 16|        |25 PC3 (D20) MAX_SS
//  EN2A (D16) PD3 17|        |24 PC2 (D19) MAX_INT
// PWM1B (D17) PD4 18|        |23 PC1 (D13) SDA
// PWM1A (D18) PD5 19|        |22 PC0 (D14) SCL
//  PWM2B (D3) PD6 20|        |21 PD7 (D4) PWM2A
//                   +--------+
//

// ATMEL ATMEGA1284P / Balanduino revision 1.3
//
//                   +---\/---+
//  EN1A (D25) PB0  1|        |40 PA0 (A0 / D7)
//  EN1B (D26) PB1  2|        |39 PA1 (A1 / D8)
//   M2A (D15) PB2  3|        |38 PA2 (A2 / D9)
//   M2B (D16) PB3  4|        |37 PA3 (A3 / D10)
//    LED (D6) PB4  5|        |36 PA4 (A4 / D11) BUZZER
//  MOSI (D27) PB5  6|        |35 PA5 (A5 / D12) VBAT
//  MISO (D28) PB6  7|        |34 PA6 (D30) EN2A
//   SCK (D29) PB7  8|        |33 PA7 (D31) EN2B
//             RST  9|        |32 AREF
//             VCC 10|        |31 GND
//             GND 11|        |30 AVCC
//           XTAL2 12|        |29 PC7 (D24) M1B
//           XTAL1 13|        |28 PC6 (D23) M1A
//     RX (D0) PD0 14|        |27 PC5 (D22) M2EN
//     TX (D1) PD1 15|        |26 PC4 (D21) M1EN
//    RX1 (D2) PD2 16|        |25 PC3 (D20) MAX_SS
//    TX1 (D3) PD3 17|        |24 PC2 (D19) MAX_INT
// PWM1B (D17) PD4 18|        |23 PC1 (D13) SDA
// PWM1A (D18) PD5 19|        |22 PC0 (D14) SCL
//  PWM2B (D4) PD6 20|        |21 PD7 (D5) PWM2A
//                   +--------+
//

#define NUM_DIGITAL_PINS            26
#define NUM_ANALOG_INPUTS           6

#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 7 : -1)
#define analogPinToChannel(p)       ((p < 6) ? (p) : (p) - 7)

#if BALANDUINO_REVISION < 13
    #define digitalPinHasPWM(p)         ((((p) >= 3) && ((p) <= 6)) || (((p) >= 17) && ((p) <= 18)))
#else
    #define digitalPinHasPWM(p)         ((((p) >= 4) && ((p) <= 6)) || (((p) >= 16) && ((p) <= 18)))
#endif

#define digitalPinToPCICR(p)        ( (((p) >= 0) && ((p) <= 31)) ? (&PCICR) : ((uint8_t *)0) )

#if BALANDUINO_REVISION < 13
    #define digitalPinToPCICRbit(p)     ( ((((p) >= 7) && ((p) <= 12)) || (((p) >= 30) && ((p) <= 31))) ? 0 : \
                                        ( (((p) == 2) || (((p) >= 5) && ((p) <= 6)) || (((p) >= 25) && ((p) <= 29))) ? 1 : \
                                        ( ((((p) >= 13) && ((p) <= 14)) || (((p) >= 19) && ((p) <= 24))) ? 2 : \
                                        ( ((((p) >= 0) && ((p) <= 1)) || (((p) >= 3) && ((p) <= 4)) || (((p) >= 15) && ((p) <= 18))) ? 3 : \
                                        0 ) ) ) )
#else
    #define digitalPinToPCICRbit(p)     ( ((((p) >= 7) && ((p) <= 12)) || (((p) >= 30) && ((p) <= 31))) ? 0 : \
                                        ( (((p) == 6) || (((p) >= 15) && ((p) <= 16)) || (((p) >= 25) && ((p) <= 29))) ? 1 : \
                                        ( ((((p) >= 13) && ((p) <= 14)) || (((p) >= 19) && ((p) <= 24))) ? 2 : \
                                        ( ((((p) >= 0) && ((p) <= 5)) || (((p) >= 17) && ((p) <= 18))) ? 3 : \
                                        0 ) ) ) )
#endif

#if BALANDUINO_REVISION < 13
    #define digitalPinToPCMSK(p)        ( ((((p) >= 7) && ((p) <= 12)) || (((p) >= 30) && ((p) <= 31))) ? (&PCMSK0) : \
                                        ( (((p) == 2) || (((p) >= 5) && ((p) <= 6)) || (((p) >= 25) && ((p) <= 29))) ? (&PCMSK1) : \
                                        ( ((((p) >= 13) && ((p) <= 14)) || (((p) >= 19) && ((p) <= 24))) ? (&PCMSK2) : \
                                        ( ((((p) >= 0) && ((p) <= 1)) || (((p) >= 3) && ((p) <= 4)) || (((p) >= 15) && ((p) <= 18))) ? (&PCMSK3) : \
                                        ((uint8_t *)0) ) ) ) )
#else
    #define digitalPinToPCMSK(p)        ( ((((p) >= 7) && ((p) <= 12)) || (((p) >= 30) && ((p) <= 31))) ? (&PCMSK0) : \
                                        ( (((p) == 6) || (((p) >= 15) && ((p) <= 16)) || (((p) >= 25) && ((p) <= 29))) ? (&PCMSK1) : \
                                        ( ((((p) >= 13) && ((p) <= 14)) || (((p) >= 19) && ((p) <= 24))) ? (&PCMSK2) : \
                                        ( ((((p) >= 0) && ((p) <= 5)) || (((p) >= 17) && ((p) <= 18))) ? (&PCMSK3) : \
                                        ((uint8_t *)0) ) ) ) )
#endif

#if BALANDUINO_REVISION < 13
    #define digitalPinToPCMSKbit(p)     ( (((p) >= 7) && ((p) <= 12)) ? ((p) - 7) : \
                                        ( (((p) >= 30) && ((p) <= 31)) ? ((p) - 24) : \
                                        ( ((p) == 2) ? (p) : \
                                        ( (((p) >= 5) && ((p) <= 6)) ? ((p) - 2) : \
                                        ( (((p) >= 25) && ((p) <= 26)) ? ((p) - 25) : \
                                        ( (((p) >= 27) && ((p) <= 29)) ? ((p) - 22) : \
                                        ( (((p) >= 13) && ((p) <= 14)) ? (14 - (p)) : \
                                        ( (((p) >= 19) && ((p) <= 24)) ? ((p) - 17) : \
                                        ( (((p) >= 0) && ((p) <= 1)) ? (p) : \
                                        ( (((p) >= 3) && ((p) <= 4)) ? ((p) + 3) : \
                                        ( (((p) >= 15) && ((p) <= 18)) ? ((p) - 13) : \
                                        0 ) ) ) ) ) ) ) ) ) ) )
#else
    #define digitalPinToPCMSKbit(p)     ( (((p) >= 7) && ((p) <= 12)) ? ((p) - 7) : \
                                        ( (((p) >= 30) && ((p) <= 31)) ? ((p) - 24) : \
                                        ( (((p) >= 0) && ((p) <= 3)) ? (p) : \
                                        ( (((p) >= 17) && ((p) <= 18)) ? (p - 13) : \
                                        ( (((p) >= 4) && ((p) <= 5)) ? ((p) + 2) : \
                                        ( (((p) >= 13) && ((p) <= 14)) ? (14 - (p)) : \
                                        ( (((p) >= 19) && ((p) <= 24)) ? ((p) - 17) : \
                                        ( (((p) >= 25) && ((p) <= 26)) ? ((p) - 25) : \
                                        ( (((p) >= 15) && ((p) <= 16)) ? ((p) - 13) : \
                                        ( ((p) == 6) ? (p - 2) : \
                                        ( (((p) >= 27) && ((p) <= 29)) ? ((p) - 22) : \
                                        0 ) ) ) ) ) ) ) ) ) ) )
#endif

#if BALANDUINO_REVISION < 13
    #define digitalPinToInterrupt(p) ((p) == 15 ? 0 : ((p) == 16 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
#else
    #define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) == 15 ? 2 : NOT_AN_INTERRUPT)))
#endif

#ifdef ARDUINO_MAIN
// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &DDRA,
        (uint16_t) &DDRB,
        (uint16_t) &DDRC,
        (uint16_t) &DDRD
};

const uint16_t PROGMEM port_to_output_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PORTA,
        (uint16_t) &PORTB,
        (uint16_t) &PORTC,
        (uint16_t) &PORTD
};
const uint16_t PROGMEM port_to_input_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PINA,
        (uint16_t) &PINB,
        (uint16_t) &PINC,
        (uint16_t) &PIND
};
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        /* User available pins */
        PD, /* 0  - PD0 */
        PD, /* 1  - PD1 */
#if BALANDUINO_REVISION < 13
        PB, /* 2  - PB2 */
        PD, /* 3  - PD6 */
        PD, /* 4  - PD7 */
        PB, /* 5  - PB3 */
#else
        PD, /* 2  - PD2 */
        PD, /* 3  - PD3 */
        PD, /* 4  - PD6 */
        PD, /* 5  - PD7 */
#endif
        PB, /* 6  - PB4 */
        PA, /* 7  - PA0 */
        PA, /* 8  - PA1 */
        PA, /* 9  - PA2 */
        PA, /* 10 - PA3 */
        PA, /* 11 - PA4 */
        PA, /* 12 - PA5 */
        PC, /* 13 - PC1 */
        PC, /* 14 - PC0 */

        /* Internal pins */
#if BALANDUINO_REVISION < 13
        PD, /* 15 - PD2 */
        PD, /* 16 - PD3 */
#else
        PB, /* 15 - PB2 */
        PB, /* 16 - PB3 */
#endif
        PD, /* 17 - PD4 */
        PD, /* 18 - PD5 */

        PC, /* 19 - PC2 */
        PC, /* 20 - PC3 */
        PC, /* 21 - PC4 */
        PC, /* 22 - PC5 */
        PC, /* 23 - PC6 */
        PC, /* 24 - PC7 */

        PB, /* 25 - PB0 */
        PB, /* 26 - PB1 */
        PB, /* 27 - PB5 */
        PB, /* 28 - PB6 */
        PB, /* 29 - PB7 */

        PA, /* 30 - PA6 */
        PA  /* 31 - PA7 */
};
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        /* User available pins */
        _BV(0), /* 0  - PD0 */
        _BV(1), /* 1  - PD1 */
#if BALANDUINO_REVISION < 13
        _BV(2), /* 2  - PB2 */
        _BV(6), /* 3  - PD6 */
        _BV(7), /* 4  - PD7 */
        _BV(3), /* 5  - PB3 */
#else
        _BV(2), /* 2  - PD2 */
        _BV(3), /* 3  - PD3 */
        _BV(6), /* 4  - PD6 */
        _BV(7), /* 5  - PD7 */
#endif
        _BV(4), /* 6  - PB4 */
        _BV(0), /* 7  - PA0 */
        _BV(1), /* 8  - PA1 */
        _BV(2), /* 9  - PA2 */
        _BV(3), /* 10 - PA3 */
        _BV(4), /* 11 - PA4 */
        _BV(5), /* 12 - PA5 */
        _BV(1), /* 13 - PC1 */
        _BV(0), /* 14 - PC0 */

        /* Internal pins */
#if BALANDUINO_REVISION < 13
        _BV(2), /* 15 - PD2 */
        _BV(3), /* 16 - PD3 */
#else // This actually ends up being the same
        _BV(2), /* 15 - PB2 */
        _BV(3), /* 16 - PB3 */
#endif
        _BV(4), /* 17 - PD4 */
        _BV(5), /* 18 - PD5 */

        _BV(2), /* 19 - PC2 */
        _BV(3), /* 20 - PC3 */
        _BV(4), /* 21 - PC4 */
        _BV(5), /* 22 - PC5 */
        _BV(6), /* 23 - PC6 */
        _BV(7), /* 24 - PC7 */

        _BV(0), /* 25 - PB0 */
        _BV(1), /* 26 - PB1 */
        _BV(5), /* 27 - PB5 */
        _BV(6), /* 28 - PB6 */
        _BV(7), /* 29 - PB7 */

        _BV(6), /* 30 - PA6 */
        _BV(7)  /* 31 - PA7 */
};
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        /* User available pins */
        NOT_ON_TIMER, /* 0  - PD0 */
        NOT_ON_TIMER, /* 1  - PD1 */
#if BALANDUINO_REVISION < 13
        NOT_ON_TIMER, /* 2  - PB2 */
        TIMER2B,      /* 3  - PD6 */
        TIMER2A,      /* 4  - PD7 */
        TIMER0A,      /* 5  - PB3 */
#else
        NOT_ON_TIMER, /* 2  - PD2 */
        NOT_ON_TIMER, /* 3  - PD3 */
        TIMER2B,      /* 4  - PD6 */
        TIMER2A,      /* 5  - PD7 */
#endif
        TIMER0B,      /* 6  - PB4 */
        NOT_ON_TIMER, /* 7  - PA0 */
        NOT_ON_TIMER, /* 8  - PA1 */
        NOT_ON_TIMER, /* 9  - PA2 */
        NOT_ON_TIMER, /* 10 - PA3 */
        NOT_ON_TIMER, /* 11 - PA4 */
        NOT_ON_TIMER, /* 12 - PA5 */
        NOT_ON_TIMER, /* 13 - PC1 */
        NOT_ON_TIMER, /* 14 - PC0 */

        /* Internal pins */
#if BALANDUINO_REVISION < 13
        NOT_ON_TIMER, /* 15 - PD2 */
        NOT_ON_TIMER, /* 16 - PD3 */
#else
        NOT_ON_TIMER, /* 15 - PB2 */
        TIMER0A,      /* 16 - PB3 */
#endif
        TIMER1B,      /* 17 - PD4 */
        TIMER1A,      /* 18 - PD5 */

        NOT_ON_TIMER, /* 19 - PC2 */
        NOT_ON_TIMER, /* 20 - PC3 */
        NOT_ON_TIMER, /* 21 - PC4 */
        NOT_ON_TIMER, /* 22 - PC5 */
        NOT_ON_TIMER, /* 23 - PC6 */
        NOT_ON_TIMER, /* 24 - PC7 */

        NOT_ON_TIMER, /* 25 - PB0 */
        NOT_ON_TIMER, /* 26 - PB1 */
        NOT_ON_TIMER, /* 27 - PB5 */
        TIMER3A,      /* 28 - PB6 */
        TIMER3B,      /* 29 - PB7 */

        NOT_ON_TIMER, /* 30 - PA6 */
        NOT_ON_TIMER  /* 31 - PA7 */
};
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.

#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#if BALANDUINO_REVISION >= 13
    #define SERIAL_PORT_HARDWARE1      Serial1
    #define SERIAL_PORT_HARDWARE_OPEN  Serial1
#endif

#endif