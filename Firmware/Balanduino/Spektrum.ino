/* Copyright (C) 2013-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#ifdef ENABLE_SPEKTRUM

#define SPEKTRUM 1024 // Set to either 1024 or 2048 depending on how it should bind with the receiver

#define RC_CHANS 12
#define SPEK_FRAME_SIZE 16

uint16_t rcValue[RC_CHANS] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // Interval [1000;2000]
uint8_t spekBuffer[SPEK_FRAME_SIZE]; // Buffer used to store the serial data from the satellite receiver
uint8_t spekIndex; // Buffer index
uint32_t spekTimer; // Used to check the time between messages, this is used to check if we started in the middle of a frame

#if (SPEKTRUM == 1024) // Frame every 22ms - 7 channels
  #define SPEK_CHAN_SHIFT 2    // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_CHAN_MASK  0x03 // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_DATA_SHIFT      // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_BIND_PULSES 3
#elif (SPEKTRUM == 2048) // Frame every 11ms - 8 channels
  #define SPEK_CHAN_SHIFT 3    // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_CHAN_MASK  0x07 // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_DATA_SHIFT >> 1 // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_BIND_PULSES 5
#endif

// Bind code inspired by David Thompson: https://code.google.com/p/nextcopterplus/source/browse/trunk/OpenAero2/src/misc_asm.S
void bindSpektrum() {
  spektrumBindPin::Clear();
  delay(63); // Tweaked until bind pulses was about 68ms after power-up
  spektrumBindPin::SetDirWrite();
  cli();
  for (uint8_t i = 0; i < SPEK_BIND_PULSES; i++) {
    spektrumBindPin::Clear();
    delayMicroseconds(118);
    spektrumBindPin::Set();
    delayMicroseconds(122);
  }
  sei();
  spektrumBindPin::SetDirRead();
  spektrumBindPin::Set();

  cfg.bindSpektrum = false;
  updateConfig(); // Update EEPROM

  buzzer::Set(); // Turn on buzzer
}

// Inspired by: https://github.com/multiwii/multiwii-firmware and https://code.google.com/p/nextcopterplus/source/browse/trunk/OpenAero2/src/isr.c
void readSpektrum(uint8_t input) {
  if (spekIndex > 0 && millis() - spekTimer > 10) // More than 10ms since last incomplete message
    spekIndex = 0; // It must be a new frame
  spekTimer = millis();

  spekBuffer[spekIndex++] = input;

  if (spekIndex == SPEK_FRAME_SIZE) { // A complete frame? If not, we'll catch it next time we are called.
    // Skip first header bytes
    for (uint8_t i = 2; i < SPEK_FRAME_SIZE; i += 2) {
      uint8_t bh = spekBuffer[i];
      uint8_t bl = spekBuffer[i + 1];
      uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
      if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
    }
    spekIndex = 0;
    spekConnected = true;
    spekConnectedTimer = millis();
#if 0 // Set this to 1 to print the channel values
    for (uint8_t i = 0; i < RC_CHANS; i++) {
      Serial.print(rcValue[i]);
      Serial.write('\t');
    }
    Serial.println(Serial.available());
#endif
  }
}

#endif // ENABLE_SPEKTRUM
