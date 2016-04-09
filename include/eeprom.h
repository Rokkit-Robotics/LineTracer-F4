#ifndef INCLUDE_EEPPROM_H
#define INCLUDE_EEPPROM_H

#include <stdint.h>

int eeprom_load(void);
int eeprom_save(void);

uint8_t *eeprom_getptr(int offset);

void eeprom_write_u8(int address, uint8_t value);
void eeprom_write_8(int address, int8_t value);
void eeprom_write_u16(int address, uint16_t value);
void eeprom_write_16(int address, int16_t value);
void eeprom_write_f(int address, float value);

uint8_t eeprom_read_u8(int address);
int8_t eeprom_read_8(int address);
uint16_t eeprom_read_u16(int address);
int16_t eeprom_read_16(int address);
float eeprom_read_f(int address);

#endif
