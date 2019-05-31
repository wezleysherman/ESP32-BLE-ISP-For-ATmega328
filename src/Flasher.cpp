#include <Arduino.h>
#include "Flasher.h"

int pmode = 0;
SPISettings fuses_spisettings = SPISettings(100000, MSBFIRST, SPI_MODE0);
SPISettings flash_spisettings = SPISettings(100000, MSBFIRST, SPI_MODE0);
unsigned char flash_state = 0;
byte image_progfuses[4] = {0x3f, 0xff, 0xde, 0x05};//{0, 0xff, 0xde, 0x05};
uint16_t pageaddr = 0;
uint8_t pagesize = 128;//pgm_read_byte(&targetimage->image_pagesize);
uint16_t chipsize = 32768;//pgm_read_word(&targetimage->chipsize);
uint8_t page_idx = 0;
byte pageBuffer[128]; /* One page of flash */

byte* flashAtmega(byte* hextext) {
	switch(flash_state) {
		case 0:
			{
				target_poweron();
				uint16_t signature;
				signature = readSignature();
				if(!signature) return nullptr;
				eraseChip();
				uint16_t out = programFuses(image_progfuses);
				end_pmode();
				start_pmode();
				pageaddr = 0;
				pagesize = 128;//pgm_read_byte(&targetimage->image_pagesize);
				chipsize = 32474;//pgm_read_word(&targetimage->chipsize);
				page_idx = 0;
				for (uint8_t i=0; i<pagesize; i++)
					pageBuffer[i] = 0xFF;
				flash_state = 1;
			}
			break;
		case 1:
			{
				if (pageaddr < chipsize && hextext) {   
         // Serial.print("Page: ");
        //  Serial.println(pageaddr);
					uint16_t len;
					byte *beginning = hextext;

					byte b, cksum = 0;

					uint16_t lineaddr;

          /*
					// Strip leading whitespace
					byte c;
					do {
						pgm_read_byte(hextext++);
					} while (c == ' ' || c == '\n' || c == '\t');

					// read one line!
					if (c != ':') {
						flash_state = 0;
						hextext = nullptr;
            Serial.println("Missing colon");
						break;
					}*/
         // byte c;
         // do {
          //  c = pgm_read_byte(hextext++);
         // } while (c == ' ' || c == '\n' || c == '\t');
					// Read the byte count into 'len'
          //Serial.println("Stuff: ");
         // Serial.println(hextext, DEC);
          //Serial.println(pgm_read_byte(hextext), DEC);
					len = *hextext++;
					//len = (len<<4) + hexton(pgm_read_byte(hextext++));
					cksum = len;

					// read high address byte
					b = *hextext++;  
					//b = (b<<4) + hexton(pgm_read_byte(hextext++));
					cksum += b;
					lineaddr = b;

					// read low address byte
					b = *hextext++; 
					//b = (b<<4) + hexton(pgm_read_byte(hextext++));
					cksum += b;
					lineaddr = (lineaddr << 8) + b;
          Serial.println(lineaddr);
          Serial.println(pageaddr);
					if (lineaddr >= (pageaddr + pagesize)) {
						hextext = beginning;
						flash_state = 3;
            Serial.println("Linaddr err");
						break;
					}

					b = *hextext++; // record type 
				//	b = (b<<4) + hexton(pgm_read_byte(hextext++));
					cksum += b;

					if (b == 0x1) { 
						// Err
						flash_state = 3;
            Serial.println("Record type error");
						break;
					} 

					for (byte i = 0; i < len; i++) {
						b = *hextext++;
						//b = (b<<4) + hexton(pgm_read_byte(hextext++));

						cksum += b;

						pageBuffer[page_idx] = b;
						page_idx++;

						if (page_idx >= pagesize) {
							boolean blankpage = true;
							for (uint8_t i=0; i<pagesize; i++)
								if (pageBuffer[i] != 0xFF) blankpage = false;

							if (! blankpage) {
								if (! flashPage(pageBuffer, pageaddr, pagesize))  {}
							}	
							page_idx = 0;
							pageaddr += pagesize;
							for (uint8_t i=0; i<pagesize; i++)
								pageBuffer[i] = 0xFF;
						}
					}
					b = *hextext++;  // chxsum
				//	b = (b<<4) + hexton(pgm_read_byte(hextext++));
					cksum += b;
					if (cksum != 0) { // Err
					}
				/*	if (pgm_read_byte(hextext++) != '\n') { // Err
						flash_state = 3;
            Serial.println("new line error");
						break;
					}*/
					if((pagesize - page_idx) < 16)  flash_state = 3; // OK
					if (page_idx == pagesize) // OK
						flash_state = 3;
				} else {
					flash_state = 3;
				}	
			}
			break;
		case 3:
			{
				boolean blankpage = true;
				for (uint8_t i=0; i<pagesize; i++) {
					if (pageBuffer[i] != 0xFF) blankpage = false;
				}          
				if (! blankpage) {
					if (!flashPage(pageBuffer, pageaddr, pagesize))  {}
				}

				programFuses(image_progfuses);
				delay(100);
				end_pmode();
				delay(100);
				start_pmode();
				delay(100);
				pageaddr = 0;
				target_poweroff();
				flash_state = 0;
				hextext = nullptr;
        digitalWrite(RESET, HIGH);  // reset it right away.
        pinMode(RESET, OUTPUT);
			}
			break;
	}
	return hextext;
}
/*
  target_poweron();
  uint16_t signature;
  signature = readSignature();
  if(!signature) return;
  eraseChip();
  byte image_progfuses[4] = {0, 0xff, 0xde, 0x05};//{0, 0xff, 0xde, 0x05};
  uint16_t out = programFuses(image_progfuses);
  end_pmode();
  start_pmode();
  byte pageBuffer[128]; /* One page of flash */
  //byte flash[image.length()];
  //image.getBytes(flash, image.length());
/*
  uint16_t pageaddr = 0;
  uint8_t pagesize = 128;//pgm_read_byte(&targetimage->image_pagesize);
  uint16_t chipsize = 16237;//pgm_read_word(&targetimage->chipsize);
  uint8_t page_idx = 0;

  for (uint8_t i=0; i<pagesize; i++)
    pageBuffer[i] = 0xFF;

  while (pageaddr < chipsize && hextext) {   
    uint16_t len;
    byte *beginning = hextext;
    
    byte b, cksum = 0;

    uint16_t lineaddr;

    // Strip leading whitespace
    byte c;
    do {
      c = pgm_read_byte(hextext++);
    } while (c == ' ' || c == '\n' || c == '\t');

      // read one line!
    if (c != ':') {
      break;
    }
    // Read the byte count into 'len'
    len = hexton(pgm_read_byte(hextext++));
    len = (len<<4) + hexton(pgm_read_byte(hextext++));
    cksum = len;
    
    // read high address byte
    b = hexton(pgm_read_byte(hextext++));  
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    lineaddr = b;
    
    // read low address byte
    b = hexton(pgm_read_byte(hextext++)); 
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    lineaddr = (lineaddr << 8) + b;
    
    if (lineaddr >= (pageaddr + pagesize)) {
      hextext = beginning;
      break;
    }

    b = hexton(pgm_read_byte(hextext++)); // record type 
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;

    if (b == 0x1) { 
     hextext = nullptr;
     break;
    } 

    for (byte i = 0; i < len; i++) {
      b = hexton(pgm_read_byte(hextext++));
      b = (b<<4) + hexton(pgm_read_byte(hextext++));

      cksum += b;

      pageBuffer[page_idx] = b;
      page_idx++;

      if (page_idx >= pagesize) {
        boolean blankpage = true;
        for (uint8_t i=0; i<pagesize; i++)
           if (pageBuffer[i] != 0xFF) blankpage = false;
                    
        if (! blankpage) {
          if (! flashPage(pageBuffer, pageaddr, pagesize))  {}
        }
        page_idx = 0;
        pageaddr += pagesize;
        for (uint8_t i=0; i<pagesize; i++)
          pageBuffer[i] = 0xFF;
      }
    }
    b = hexton(pgm_read_byte(hextext++));  // chxsum
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    if (cksum != 0) {
    }
    if (pgm_read_byte(hextext++) != '\n') {
      break;
    }
    if((pagesize - page_idx) < 16) break;
    if (page_idx == pagesize) 
      break;
  }
  boolean blankpage = true;
  for (uint8_t i=0; i<pagesize; i++) {
     if (pageBuffer[i] != 0xFF) blankpage = false;
     
   }          
   if (! blankpage) {
     if (! flashPage(pageBuffer, pageaddr, pagesize))  {}
   }

  programFuses(image_progfuses);
  delay(100);
  end_pmode();
  delay(100);
  start_pmode();
  delay(100);
  pageaddr = 0;
  target_poweroff();
}

*/
/*
 * readSignature
 * read the bottom two signature bytes (if possible) and return them
 * Note that the highest signature byte is the same over all AVRs so we skip it
 */

uint16_t readSignature (void)
{
    
  uint16_t target_type = 0;
  
  SPI.beginTransaction(fuses_spisettings); 
  
  target_type = spi_transaction(0x30, 0x00, 0x01, 0x00);
  target_type <<= 8;
  target_type |= spi_transaction(0x30, 0x00, 0x02, 0x00);

  SPI.endTransaction();
  
  if (target_type == 0 || target_type == 0xFFFF) {
    if (target_type == 0) {
    }
  }
  return target_type;
}

/*
 * programmingFuses
 * Program the fuse/lock bits
 */
boolean programFuses(const byte *fuses)
{
    
  byte f;

  busyWait();
  f = pgm_read_byte(&fuses[FUSE_PROT]);
  if (f) {
    SPI.beginTransaction(fuses_spisettings);
    spi_transaction(0xAC, 0xE0, 0x00, f);
    SPI.endTransaction();
  }
  busyWait();
  f = pgm_read_byte(&fuses[FUSE_LOW]);
  if (f) {
    SPI.beginTransaction(fuses_spisettings);
    spi_transaction(0xAC, 0xA0, 0x00, f);
    SPI.endTransaction();
  }
  busyWait();
  f = pgm_read_byte(&fuses[FUSE_HIGH]);
  if (f) {
    SPI.beginTransaction(fuses_spisettings);
    spi_transaction(0xAC, 0xA8, 0x00, f);
    SPI.endTransaction();
  }
  busyWait();
  f = pgm_read_byte(&fuses[FUSE_EXT]);
  if (f) {
    SPI.beginTransaction(fuses_spisettings);
    spi_transaction(0xAC, 0xA4, 0x00, f);
    SPI.endTransaction();
  }    
  return true;      /* */
}


/*
 * readImagePage
 *
 * Read a page of intel hex image from a string in pgm memory.
*/

// Returns number of bytes decoded
byte * readImagePage (byte *hextext, uint16_t pageaddr, uint8_t pagesize, byte *page)
{
  
  uint16_t len;
  uint8_t page_idx = 0;
  byte *beginning = hextext;
  
  byte b, cksum = 0;

  //Serial.print("page size = "); Serial.println(pagesize, DEC);

  // 'empty' the page by filling it with 0xFF's
  for (uint8_t i=0; i<pagesize; i++)
    page[i] = 0xFF;

  while (1) {
    uint16_t lineaddr;

    // Strip leading whitespace
    byte c;
    do {
      c = pgm_read_byte(hextext++);
    } while (c == ' ' || c == '\n' || c == '\t');

      // read one line!
    if (c != ':') {
      break;
    }
    // Read the byte count into 'len'
    len = hexton(pgm_read_byte(hextext++));
    len = (len<<4) + hexton(pgm_read_byte(hextext++));
    cksum = len;
    
    // read high address byte
    b = hexton(pgm_read_byte(hextext++));  
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    lineaddr = b;
    
    // read low address byte
    b = hexton(pgm_read_byte(hextext++)); 
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    lineaddr = (lineaddr << 8) + b;
    
    if (lineaddr >= (pageaddr + pagesize)) {
      return beginning;
    }

    b = hexton(pgm_read_byte(hextext++)); // record type 
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    //Serial.print("Record type "); Serial.println(b, HEX);
    if (b == 0x1) { 
     // end record, return nullptr to indicate we're done
     hextext = nullptr;
     break;
    } 

    for (byte i=0; i < len; i++) {
      // read 'n' bytes
    //  if(i < len) {
        b = hexton(pgm_read_byte(hextext++));
        b = (b<<4) + hexton(pgm_read_byte(hextext++));
     // } else {
     //   b = 0xFF;
      //}
      
      cksum += b;

      page[page_idx] = b;
      page_idx++;

      if (page_idx > pagesize) {
          break;
      }
    }
    b = hexton(pgm_read_byte(hextext++));  // chxsum
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    if (cksum != 0) {
    }
    if (pgm_read_byte(hextext++) != '\n') {
      break;
    }
    if((pagesize - page_idx) < 16) break;
    if (page_idx == pagesize) 
      break;
  }
  return hextext;
}
bool flash_a = false;
// Send one byte to the page buffer on the chip
void flashWord (uint8_t hilo, uint16_t addr, uint8_t data) {
  if(!flash_a) {
    ledcWrite(0, 100); 
  } else {
    ledcWrite(0, 0);  
  }
  flash_a = !flash_a;
  spi_transaction(0x40+8*hilo, addr>>8 & 0xFF, addr & 0xFF, data);
}

// Basically, write the pagebuff (with pagesize bytes in it) into page $pageaddr
boolean flashPage (byte *pagebuff, uint16_t pageaddr, uint8_t pagesize) {  

  SPI.beginTransaction(flash_spisettings);
  for (uint16_t i=0; i < pagesize/2; i++) {
    flashWord(LOW, i, pagebuff[2*i]);
    flashWord(HIGH, i, pagebuff[2*i+1]);
  }

  // page addr is in bytes, byt we need to convert to words (/2)
  pageaddr = (pageaddr/2) & 0xFFC0;

  uint16_t commitreply = spi_transaction(0x4C, (pageaddr >> 8) & 0xFF, pageaddr & 0xFF, 0);

  SPI.endTransaction();

  if (commitreply != pageaddr) 
    return false;

  busyWait();
  
  
  return true;
}

void start_pmode() {
  pinMode(13, INPUT); // restore to default

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  SPI.end();
  digitalWrite(MISO, LOW);    /* Make sure pullups are off too */
  pinMode(MISO, INPUT);
  digitalWrite(MOSI, LOW);
  pinMode(MOSI, INPUT);
  digitalWrite(SCK, LOW);
  pinMode(SCK, INPUT);
  digitalWrite(RESET, LOW);
  pinMode(RESET, INPUT);
  pmode = 0;
}


/*
 * target_poweron
 * begin programming
 */
boolean target_poweron()
{
  pinMode(LED_PROGMODE, OUTPUT);
  digitalWrite(LED_PROGMODE, HIGH);
  digitalWrite(RESET, LOW);  // reset it right away.
  pinMode(RESET, OUTPUT);
  delay(100);
  start_pmode();
  return true;
}

boolean target_poweroff()
{
  end_pmode();
  digitalWrite(LED_PROGMODE, LOW);
  digitalWrite(RESET, HIGH);  // reset it right away.
  pinMode(RESET, OUTPUT);
  return true;
}

// Send the erase command, then busy wait until the chip is erased
void eraseChip(void) {
  SPI.beginTransaction(fuses_spisettings);
  spi_transaction(0xAC, 0x80, 0, 0);  // chip erase    
  SPI.endTransaction();
  busyWait();
}

// Simply polls the chip until it is not busy any more - for erasing and programming
void busyWait(void)  {
  SPI.beginTransaction(fuses_spisettings);
  byte busybit;
  do {
    busybit = spi_transaction(0xF0, 0x0, 0x0, 0x0);
  } while (busybit & 0x01);
  SPI.endTransaction();
}


/*
 * Functions specific to ISP programming of an AVR
 */
uint16_t spi_transaction (uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n, m, r;
  SPI.transfer(a); 
  n = SPI.transfer(b);
  m = SPI.transfer(c);
  r = SPI.transfer(d);
  return 0xFFFFFF & (((uint32_t)n<<16)+(m<<8) + r);
}

byte hexton (byte h)
{
  if (h >= '0' && h <= '9')
    return(h - '0');
  if (h >= 'A' && h <= 'F')
    return((h - 'A') + 10);
}
