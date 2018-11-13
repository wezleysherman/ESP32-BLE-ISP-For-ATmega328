/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include "optiLoader.h"
#include "SPI.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool transmit = false;
boolean target_poweroff ();
void end_pmode();
void start_pmode();
bool receiveImage = false;
int pmode=0;
String image = "";
void flashAtmega();
boolean target_poweron ();
uint8_t txValue = 0;
uint8_t out_buff[6];
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID_RX "80c78362-e3f9-11e8-9f32-f2801f1b9fd1"
#define CHARACTERISTIC_UUID_TX "80c784ac-e3f9-11e8-9f32-f2801f1b9fd1"
unsigned char checksum;
#define SCK 18
#define MISO 19
#define MOSI 23
#define LED_PROGMODE 17
#define RESET 22

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};



class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      String input = ""; 
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Vaue: ");
        for (int i = 0; i < rxValue.length(); i++)
          if(i % 2 == 0)
            input += rxValue[i];
          
        //Serial.print(input);
        if(receiveImage == true) {
          Serial.println("SHDM");
          Serial.println(image.substring(image.length()-5, image.length()));
          image += input;
          if(image.substring(image.length()-5, image.length()).equals("0x0FC")) {
            uint8_t outBuff1[] = "OxOFC";
            memcpy(out_buff, outBuff1, sizeof(outBuff1));
            transmit = true;
            image = image.substring(0, image.length()-5);
           // image += "    )\"}";
            Serial.println();
            Serial.println(image);
            flashAtmega();
            receiveImage = false;
            return;
          }
          Serial.println();
          Serial.println(image);
          Serial.println(image.length());
        }
        Serial.println();
        Serial.println("*********");
      
        String input2 = "0x0FA";
        if(input.equals(input2)) {
          Serial.println("Uploading Code");
          uint8_t outBuff[] = "0x0FB";
          memcpy(out_buff, outBuff, sizeof(outBuff));
          transmit = true;
          receiveImage = true;
        }
      }
    }
};


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Trynkit Husky");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
unsigned int pageIndex = 1;

unsigned char getHex(byte pageBuffer) {
  unsigned char data = pageBuffer;
  if(data >= 'A') {
    data -= 'A' - 10;
  } else {
    data -= '0';
  }

  return data;
}

unsigned char getBootData(byte pageBuffer) {
  unsigned char data;
  data = getHex(pageBuffer) << 4;
  data |= getHex(pageBuffer++);
  checksum += data;
  return data;
}


void flashAtmega() {
    target_poweron();
    uint16_t signature;
    signature = readSignature();
    Serial.println(signature); 
    if(!signature) return;
    eraseChip();
    byte image_progfuses[4] = {0, 0x62, 0xdf, 0x00};
    uint16_t out = programFuses(image_progfuses);
    Serial.println(out);
    end_pmode();
    start_pmode();
    Serial.println("Set addresses and page space!");
  byte pageBuffer[128]; /* One page of flash */

  byte flash[image.length()];
  image.getBytes(flash, image.length());
  Serial.println(image);
  Serial.println(image.length());
  byte *hextext = flash;  
  uint16_t pageaddr = 0;
  uint8_t pagesize = 128;//pgm_read_byte(&targetimage->image_pagesize);
  Serial.print("Page size: "); Serial.println(pagesize, DEC);
  uint16_t chipsize = 16237;//pgm_read_word(&targetimage->chipsize);
  Serial.print("Chip size: "); Serial.println(chipsize, DEC);
  
  while (pageaddr < chipsize && hextext) {
     Serial.print("Writing address $"); Serial.println(pageaddr, HEX);
     byte *hextextpos = readImagePage (hextext, pageaddr, pagesize, pageBuffer);
          
     boolean blankpage = true;
     for (uint8_t i=0; i<pagesize; i++) {
       if (pageBuffer[i] != 0xFF) blankpage = false;
       
     }          
     if (! blankpage) {
       if (! flashPage(pageBuffer, pageaddr, pagesize))  
         error("Flash programming failed");
     } else {
      error("we has issues of blank pages");
     }
     hextext = hextextpos;
     pageaddr += pagesize;
}
   
    programFuses(image_progfuses);
    delay(100);
    end_pmode();
    delay(100);
    start_pmode();
    delay(100);
    pageaddr = 0;
    
    target_poweroff();
    image = "";
}

void loop() {
    if (deviceConnected) {
      // bluetooth stack will go into congestion, if too many packets are sent
      if(transmit == true) {
        pTxCharacteristic->setValue(out_buff, 5);
        pTxCharacteristic->notify();
        delay(10);
        uint8_t empty[6];
        transmit = false;
        memcpy(out_buff, empty, sizeof(empty));
      }
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
        
        
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

void start_pmode () {
  pinMode(13, INPUT); // restore to default

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  
  debug("...spi_init done");
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
  debug("...spi_transaction");
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  debug("...Done");
  pmode = 1;
}

void end_pmode () {
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
boolean target_poweron ()
{
  pinMode(LED_PROGMODE, OUTPUT);
  digitalWrite(LED_PROGMODE, HIGH);
  digitalWrite(RESET, LOW);  // reset it right away.
  pinMode(RESET, OUTPUT);
  delay(100);
  Serial.print("Starting Program Mode");
  start_pmode();
  Serial.println(" [OK]");
  return true;
}

boolean target_poweroff ()
{
  end_pmode();
  digitalWrite(LED_PROGMODE, LOW);
  return true;
}

void error(const char *string) {
  Serial.println(string); 
}
