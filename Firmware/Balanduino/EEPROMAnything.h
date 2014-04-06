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

#ifndef _eepromanything_h_
#define _eepromanything_h_

#include <avr/eeprom.h>

// Source: http://playground.arduino.cc/Code/EEPROMWriteAnything
// Modified to use the standard EEPROM library and added 'eeprom_busy_wait()'
// Also added EEPROM_updateAnything to limit number of write/erase cycles
// By: Krisitan Lauszus, TKJ Electronics

template <class T> uint16_t EEPROM_writeAnything(uint16_t addr, const T& value) {
  eeprom_busy_wait(); // Wait until the EEPROM is ready
  const uint8_t *p = (const uint8_t*)(const void*)&value;
  uint16_t i;
  for (i = 0; i < sizeof(value); i++)
    eeprom_write_byte((uint8_t*)addr++, *p++);
  return i;
}

template <class T> uint16_t EEPROM_updateAnything(uint16_t addr, const T& value) {
  eeprom_busy_wait(); // Wait until the EEPROM is ready
  const uint8_t *p = (const uint8_t*)(const void*)&value;
  uint16_t i;
  for (i = 0; i < sizeof(value); i++) {
    if (eeprom_read_byte((uint8_t*)addr) != *p) // Limits number of write/erase cycles
      eeprom_write_byte((uint8_t*)addr, *p);
    addr++;
    p++;
  }
  return i;
}

template <class T> uint16_t EEPROM_readAnything(uint16_t addr, T& value) {
  eeprom_busy_wait(); // Wait until the EEPROM is ready
  uint8_t *p = (uint8_t*)(void*)&value;
  uint16_t i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = eeprom_read_byte((uint8_t*)addr++);
  return i;
}

#endif
