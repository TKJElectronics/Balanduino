/* Copyright (C) 2013 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

// Inspired by: https://github.com/multiwii/multiwii-firmware

#ifdef ENABLE_SPEKTRUM

#define SPEKTRUM 1024 // Set to either 1024 or 2048
#define RC_CHANS 12
uint16_t rcValue[RC_CHANS] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // Interval [1000;2000]

#define SPEK_FRAME_SIZE 16

#if (SPEKTRUM == 1024)
  #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_DATA_SHIFT          // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_BIND_PULSES 3
#elif (SPEKTRUM == 2048)
  #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_DATA_SHIFT >> 1     // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_BIND_PULSES 5
#endif

void readSpektrum() {
  while (Serial.available() > SPEK_FRAME_SIZE) { // More than a frame? More bytes implies we weren't called for multiple frame times. We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++) {
      Serial.read();  // Toss one full frame of bytes.
    }
  }
  if (Serial.available() == SPEK_FRAME_SIZE) { // A complete frame? If not, we'll catch it next time we are called.
    Serial.read(); Serial.read(); // Eat the header bytes
    for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2) {
      uint8_t bh = Serial.read();
      uint8_t bl = Serial.read();
      uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
      if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
    }
#if 0 // Set this to 1 to print the channel values
    for (uint8_t i = 0; i < RC_CHANS; i++) {
      Serial.print(rcValue[i]);
      Serial.write('\t');
    }
    Serial.println();
#endif
  }
}

#endif // ENABLE_SPEKTRUM
