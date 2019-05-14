#include "ECC508.h"
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
uint16_t crc16(const uint8_t data[], size_t length)
{
  if (data == NULL || length == 0) {
    return 0;
  }

  uint16_t crc = 0;

  while (length) {
    byte b = *data;

    for (uint8_t shift = 0x01; shift > 0x00; shift <<= 1) {
      uint8_t dataBit = (b & shift) ? 1 : 0;
      uint8_t crcBit = crc >> 15;

      crc <<= 1;
      
      if (dataBit != crcBit) {
        crc ^= 0x8005;
      }
    }

    length--;
    data++;
  }

  return crc;
}


int wakeup() {
  unsigned char retries = 3;
  Wire.setClock(100000u);
  Wire.beginTransmission(0x00);
  Wire.endTransmission();
  delayMicroseconds(1500);

  while(Wire.requestFrom(96, 3) != 4 && retries--);
  while(Wire.available()) {
    Wire.read();
  }

  Wire.setClock(1000000u);
  return 1;
}

int idle() {
  Wire.beginTransmission(96);
  Wire.write(0x02);
  int endT = Wire.endTransmission();
  if(endT != 0)
    return endT;
  delay(1);
  return 1;
}

String getSerial() {
  String serialNum = "";
  wakeup();
  idle();
  delay(500);
  wakeup();
  int commandLength = 8 + 0; // 1 for type, 1 for length, 1 for opcode, 1 for param1, 2 for param2, 2 for crc
  byte command[commandLength]; 
  uint16_t param2 = 0;
  size_t dataLength = 0;
  command[0] = 0x03;
  command[1] = sizeof(command) - 1;
  command[2] = 0x02;
  command[3] = 0;
  memcpy(&command[4], &param2, sizeof(param2));
  memcpy(&command[6], NULL, dataLength);

  uint16_t crc = crc16(&command[1], 8 - 3 + dataLength);
  memcpy(&command[6 + dataLength], &crc, sizeof(crc));
  Wire.beginTransmission(96);
  Wire.write(command, commandLength);
  Wire.endTransmission();

  delay(5);
  int retries = 20;
  while(Wire.requestFrom(96, 4) != 4 && retries--);
  while(Wire.available()) { 
    serialNum += Wire.read();
    serialNum += " ";
  }
 
  idle();
  delay(500);
  wakeup();
  commandLength = 8 + 0; // 1 for type, 1 for length, 1 for opcode, 1 for param1, 2 for param2, 2 for crc
  command[commandLength]; 
  param2 = 2;
  dataLength = 0;
  command[0] = 0x03;
  command[1] = sizeof(command) - 1;
  command[2] = 0x02;
  command[3] = 0;
  memcpy(&command[4], &param2, sizeof(param2));
  memcpy(&command[6], NULL, dataLength);

  crc = crc16(&command[1], 8 - 3 + dataLength);
  memcpy(&command[6 + dataLength], &crc, sizeof(crc));
  Wire.beginTransmission(96);
  Wire.write(command, commandLength);
  Wire.endTransmission();

  delay(5);
  retries = 20;
  while(Wire.requestFrom(96, 4) != 4 && retries--);
  while(Wire.available()) { 
    serialNum += Wire.read();
    serialNum += " ";
  }

  idle();
  delay(500);
  wakeup();
  commandLength = 8 + 0; // 1 for type, 1 for length, 1 for opcode, 1 for param1, 2 for param2, 2 for crc
  command[commandLength]; 
  param2 = 3;
  dataLength = 0;
  command[0] = 0x03;
  command[1] = sizeof(command) - 1;
  command[2] = 0x02;
  command[3] = 0;
  memcpy(&command[4], &param2, sizeof(param2));
  memcpy(&command[6], NULL, dataLength);

  crc = crc16(&command[1], 8 - 3 + dataLength);
  memcpy(&command[6 + dataLength], &crc, sizeof(crc));
  Wire.beginTransmission(96);
  Wire.write(command, commandLength);
  Wire.endTransmission();

  delay(5);
  retries = 20;
  while(Wire.requestFrom(96, 4) != 4 && retries--);
  while(Wire.available()) { 
    serialNum += Wire.read();
    serialNum += " ";
  }
  serialNum = serialNum.substring(0, serialNum.length()-1);
  idle();
  return serialNum;
}
