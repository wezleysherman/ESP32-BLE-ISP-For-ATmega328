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
#include <EEPROM.h>
#include <HTTPClient.h>
#include "SPI.h"
#include "soc/rtc.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include "esp_bt_main.h"
#include "ArduinoJson.h"

const char* HTTPS_CERT = "-----BEGIN CERTIFICATE-----\n" \
"MIIESTCCAzGgAwIBAgITBn+UV4WH6Kx33rJTMlu8mYtWDTANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MTAyMjAwMDAwMFoXDTI1MTAxOTAwMDAwMFowRjEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEVMBMGA1UECxMMU2VydmVyIENB\n" \
"IDFCMQ8wDQYDVQQDEwZBbWF6b24wggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK\n" \
"AoIBAQDCThZn3c68asg3Wuw6MLAd5tES6BIoSMzoKcG5blPVo+sDORrMd4f2AbnZ\n" \
"cMzPa43j4wNxhplty6aUKk4T1qe9BOwKFjwK6zmxxLVYo7bHViXsPlJ6qOMpFge5\n" \
"blDP+18x+B26A0piiQOuPkfyDyeR4xQghfj66Yo19V+emU3nazfvpFA+ROz6WoVm\n" \
"B5x+F2pV8xeKNR7u6azDdU5YVX1TawprmxRC1+WsAYmz6qP+z8ArDITC2FMVy2fw\n" \
"0IjKOtEXc/VfmtTFch5+AfGYMGMqqvJ6LcXiAhqG5TI+Dr0RtM88k+8XUBCeQ8IG\n" \
"KuANaL7TiItKZYxK1MMuTJtV9IblAgMBAAGjggE7MIIBNzASBgNVHRMBAf8ECDAG\n" \
"AQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUWaRmBlKge5WSPKOUByeW\n" \
"dFv5PdAwHwYDVR0jBBgwFoAUhBjMhTTsvAyUlC4IWZzHshBOCggwewYIKwYBBQUH\n" \
"AQEEbzBtMC8GCCsGAQUFBzABhiNodHRwOi8vb2NzcC5yb290Y2ExLmFtYXpvbnRy\n" \
"dXN0LmNvbTA6BggrBgEFBQcwAoYuaHR0cDovL2NydC5yb290Y2ExLmFtYXpvbnRy\n" \
"dXN0LmNvbS9yb290Y2ExLmNlcjA/BgNVHR8EODA2MDSgMqAwhi5odHRwOi8vY3Js\n" \
"LnJvb3RjYTEuYW1hem9udHJ1c3QuY29tL3Jvb3RjYTEuY3JsMBMGA1UdIAQMMAow\n" \
"CAYGZ4EMAQIBMA0GCSqGSIb3DQEBCwUAA4IBAQCFkr41u3nPo4FCHOTjY3NTOVI1\n" \
"59Gt/a6ZiqyJEi+752+a1U5y6iAwYfmXss2lJwJFqMp2PphKg5625kXg8kP2CN5t\n" \
"6G7bMQcT8C8xDZNtYTd7WPD8UZiRKAJPBXa30/AbwuZe0GaFEQ8ugcYQgSn+IGBI\n" \
"8/LwhBNTZTUVEWuCUUBVV18YtbAiPq3yXqMB48Oz+ctBWuZSkbvkNodPLamkB2g1\n" \
"upRyzQ7qDn1X8nn8N8V7YJ6y68AtkHcNSRAnpTitxBKjtKPISLMVCx7i4hncxHZS\n" \
"yLyKQXhw2W2Xs0qLeC1etA+jTGDK4UfLeC0SF7FSi8o5LL21L8IzApar2pR/\n" \
"-----END CERTIFICATE-----\n";

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool wifiSetup = false;
bool transmit = false;
boolean target_poweroff ();
void end_pmode();
void start_pmode();
bool receiveImage = false;
int pmode=0;
int wifiStep = 0;
String image = "";
String wifiData = "";
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
#define EEPROM_SIZE 1000
unsigned long updateTimer;

typedef struct wifi_settings {
  bool saved = true;
  char ssid[128];
  char password[128];
  char deviceID[128];
  char deviceName[128];
  char deviceKey[15];
  char buff[5];
} WIFI;

WIFI wifi_settings;



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
        Serial.println(input);
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

        if(wifiSetup == true) {
          wifi_settings.saved = false;
          wifiData += input;
          if(wifiData.substring(wifiData.length()-5, wifiData.length()).equals("0x0FC")) {
            wifiData = wifiData.substring(0, wifiData.length()-5);
            if(wifiStep == 0) {
              strcpy(wifi_settings.deviceName, wifiData.c_str()); 
              uint8_t outBuff[] = "0x0DI";
              memcpy(out_buff, outBuff, sizeof(outBuff));
              transmit = true;
            } else if(wifiStep == 1) {
              strcpy(wifi_settings.deviceID, wifiData.c_str()); 
              uint8_t outBuff[] = "0x0DS";
              memcpy(out_buff, outBuff, sizeof(outBuff));
              transmit = true;
            } else if(wifiStep == 2) {
              strcpy(wifi_settings.deviceKey, wifiData.c_str()); 
              uint8_t outBuff[] = "0x0DD";
              memcpy(out_buff, outBuff, sizeof(outBuff));
              transmit = true;
            } else if(wifiStep == 3) {
              strcpy(wifi_settings.ssid, wifiData.c_str()); 
              uint8_t outBuff[] = "0x0DW";
              memcpy(out_buff, outBuff, sizeof(outBuff));
              transmit = true;
            } else if(wifiStep == 4) {
              strcpy(wifi_settings.password, wifiData.c_str()); 
              uint8_t outBuff[] = "0x00F";
              memcpy(out_buff, outBuff, sizeof(outBuff));
              transmit = true;
              wifiStep = -1;
              wifiSetup = false;
            } else {
              wifiStep = -1;
              wifiSetup = false;
            }
            Serial.println(wifi_settings.deviceName);
            EEPROM.put(0, wifi_settings);
            EEPROM.commit();
            wifiStep ++;
            Serial.println(input);
            delay(200);
            wifiData = "";
          }
        }
        
      
        String input2 = "0x0FA";
        if(input.equals(input2)) {
          Serial.println("Uploading Code");
          uint8_t outBuff[] = "0x0FB";
          memcpy(out_buff, outBuff, sizeof(outBuff));
          transmit = true;
          receiveImage = true;
        }

        String inputWiFi = "0x0FB";
        if(input.equals(inputWiFi)) {
          Serial.println("Update WiFi");
          uint8_t outBuff1[] = "0x0DN";
          memcpy(out_buff, outBuff1, sizeof(outBuff1));
          transmit = true;
          wifiSetup = true;
        }
      }
    }
};
HTTPClient http;

void deinitBLE() {
     esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
        esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
}

void initBLE() {
  ESP.restart();
}

void setup() {
  updateTimer = millis();
 //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifi_settings);
  
   // Create the BLE Device
    Serial.println("BLE init");
  //esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  BLEDevice::init("Trynkit Husky");
  Serial.println("BLE init complete");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("Creating server forBLE");
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
  Serial.println("Created services");
  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Services started");
  // WiFi
  WiFi.mode(WIFI_STA);
  Serial.println(wifi_settings.saved);
  if(wifi_settings.saved == false) {
    Serial.println(wifi_settings.ssid);
    Serial.println(wifi_settings.password);
    Serial.println(wifi_settings.deviceID);
    Serial.println(wifi_settings.deviceName);
    Serial.println(wifi_settings.deviceKey);
    WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  }
  
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
    if(!signature) return;
    eraseChip();
    byte image_progfuses[4] = {0, 0x62, 0xdf, 0x00};
    uint16_t out = programFuses(image_progfuses);
    end_pmode();
    start_pmode();
    byte pageBuffer[128]; /* One page of flash */
  
    byte flash[image.length()];
    image.getBytes(flash, image.length());
    byte *hextext = flash;  
    uint16_t pageaddr = 0;
    uint8_t pagesize = 128;//pgm_read_byte(&targetimage->image_pagesize);
    uint16_t chipsize = 16237;//pgm_read_word(&targetimage->chipsize);
    
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

void fetchOTA() {
  deinitBLE();
  String url = "https://trynkit.us/api/fetch_ota/";
  url += wifi_settings.deviceID;
  url += "/";
  url += wifi_settings.deviceKey;
  //url = url.substring(0, url.length()-5);
  Serial.println(url);
  http.begin(url.c_str(), HTTPS_CERT);
  int httpResp = http.GET();
  if(httpResp == 200) {
    String httpResp = http.getString();
    DynamicJsonBuffer JSONBuffer;                         //Memory pool
    JsonObject& parsed = JSONBuffer.parseObject(httpResp); //Parse message
    bool hasImage = parsed["has_update"];
    String parsedOut;
    parsed.printTo(parsedOut);
    Serial.println(parsedOut);
    Serial.println(hasImage);
    if(hasImage) {
      String codeImage = parsed["code"];
      image ="";
      image += codeImage;
      image.replace("\r", "");
      Serial.println(image);
      flashAtmega();
      deleteOTA();
      receiveImage = false; 
    }
  }
  http.end();
  delay(100);
  initBLE();
}

void deleteOTA() {
  String url = "https://trynkit.us/api/delete_ota/";
  url += wifi_settings.deviceID;
  url += "/";
  url += wifi_settings.deviceKey;
  //url = url.substring(0, url.length()-5);
  Serial.println(url);
  http.begin(url.c_str(), HTTPS_CERT);
  int httpResp = http.GET();
  http.end();
}

bool fetchingOTA = false;
bool wifiFlag = false;
bool wifiConnected = false;
void loop() {
    if(wifiConnected == false && WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi Connected!" );
      wifiConnected = true;
    }

    if(WiFi.status() == WL_CONNECTED && (millis() - updateTimer) >= 15000) {
      updateTimer = millis();
      fetchOTA();
    }
    
    if (deviceConnected) {
      if(wifiFlag == false) {
        disconnectWiFi();
        delay(20);
      }
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
        oldDeviceConnected = deviceConnected;
        wifiStep = 0;
        wifiSetup = false;
        image = "";
        reconnectWiFi();
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		    wifiStep = 0;
        wifiSetup = false;
        image = "";
        oldDeviceConnected = deviceConnected;
    }
}

void disconnectWiFi() {
   wifiFlag = true;
   wifiConnected = false;
   WiFi.disconnect();
}

void reconnectWiFi() {
  wifiFlag = false;
  Serial.println(wifiConnected);
  Serial.println(wifi_settings.ssid);
  Serial.println(wifi_settings.password);
  WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  WiFi.reconnect();
  Serial.print("WiFi Reconnecting");
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
