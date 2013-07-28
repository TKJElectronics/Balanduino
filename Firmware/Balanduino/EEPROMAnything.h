#ifndef _eepromanything_h_
#define _eepromanything_h_

#include <avr/eeprom.h>

// Source: http://playground.arduino.cc/Code/EEPROMWriteAnything
// Modified to use the standard eeprom library and added 'eeprom_busy_wait()'
// Also added EEPROM_updateAnything to limit number of write/erase cycles
// By: Krisitan Lauszus, TKJ Electronics

template <class T> uint8_t EEPROM_writeAnything(uint8_t addr, const T& value) {
  eeprom_busy_wait(); // Wait until the eeprom is ready
  const uint8_t *p = (const uint8_t*)(const void*)&value;
  uint8_t i;
  for (i = 0; i < sizeof(value); i++)
    eeprom_write_byte((uint8_t*)addr++, *p++);
  return i;
}
template <class T> uint8_t EEPROM_updateAnything(uint8_t addr, const T& value) {
  eeprom_busy_wait(); // Wait until the eeprom is ready
  const uint8_t *p = (const uint8_t*)(const void*)&value;
  uint8_t i;
  for (i = 0; i < sizeof(value); i++) {
    if (eeprom_read_byte((uint8_t*)addr) != *p) // Limits number of write/erase cycles
      eeprom_write_byte((uint8_t*)addr, *p);
    addr++;
    p++;
  }
  return i;
}
template <class T> uint8_t EEPROM_readAnything(uint8_t addr, T& value) {
  eeprom_busy_wait(); // Wait until the eeprom is ready
  uint8_t *p = (uint8_t*)(void*)&value;
  uint8_t i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = eeprom_read_byte((uint8_t*)addr++);
  return i;
}

#endif
