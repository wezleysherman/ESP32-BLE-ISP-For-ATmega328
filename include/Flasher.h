#include "Arduino.h"
#include <pgmspace.h>
#include "SPI.h"
#ifndef _FLASHER_H_INCLUDED
#define _FLASHER_H_INCLUDED

#define FUSE_PROT 0      /* memory protection */
#define FUSE_LOW 1      /* Low fuse */
#define FUSE_HIGH 2     /* High fuse */
#define FUSE_EXT 3      /* Extended fuse */

#define LED_ERR 8
#define LED_PROGMODE A0

#define SCK 18
#define MISO 19
#define MOSI 23
#define RESET 14

byte* flashAtmega(byte* hextext);
void start_pmode();
void end_pmode();
boolean target_poweron();
boolean target_poweroff();
boolean programFuses(const byte *fuses);
uint16_t readSignature (void);
byte * readImagePage (byte *hextext, uint16_t pageaddr, uint8_t pagesize, byte *page);
void flashWord (uint8_t hilo, uint16_t addr, uint8_t data);
boolean flashPage (byte *pagebuff, uint16_t pageaddr, uint8_t pagesize);
uint16_t spi_transaction (uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void busyWait(void);
void eraseChip(void);
byte hexton (byte h);
#endif
