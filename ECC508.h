#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#ifndef ECC508_H_INCLUDED
#define ECC508_H_INCLUDED
uint16_t crc16(const uint8_t data[], size_t length);
String getSerial();
#endif
