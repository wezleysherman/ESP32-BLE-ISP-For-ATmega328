#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <pgmspace.h>
#include "SPI.h"

#ifndef _OPTILOADER_H
#define _OPTILOADER_H

#define FUSE_PROT 0      /* memory protection */
#define FUSE_LOW 1      /* Low fuse */
#define FUSE_HIGH 2     /* High fuse */
#define FUSE_EXT 3      /* Extended fuse */

#define LED_ERR 8
#define LED_PROGMODE A0

// Useful message printing definitions

#define debug(string) // flashprint(PSTR(string));

uint16_t spi_transaction (uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint16_t readSignature (void);
boolean programFuses (const byte *fuses);
void eraseChip(void);
boolean verifyImage (byte *hextext);
void busyWait(void);
boolean flashPage (byte *pagebuff, uint16_t pageaddr, uint8_t pagesize);
byte hexton (byte h);
byte * readImagePage (byte *hextext, uint16_t pageaddr, uint8_t pagesize, byte *page);
void error(const char *string);

#endif
