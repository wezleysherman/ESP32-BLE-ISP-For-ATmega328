#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include "Flasher.h"
#include "TrynkitFirmware.h"
#include "ReceiveCallBack.h"
#include "ArduinoJson.h"
#include <HardwareSerial.h>

void setup() {
  Serial.begin(115200);
  // Restore WiFi settings if they exist
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifi_settings);
  ATmegaSerial.begin(9600, SERIAL_8N1, 3, 1);
  //pinMode(12, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(12, 0);
  // Set WiFi OTA Timer after everything else has been initialized
  updateTimer = millis();
  WiFi.mode(WIFI_STA);
  if(wifi_settings.saved == false) {
    WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  }
  TaskHandle_t LEDTask;
  xTaskCreatePinnedToCore(updateLED, "LEDTask", 10000, NULL, 1, &LEDTask, 0);
   //Init BLE
  initBLE();
}

int serialCount = 0;
void loop() {
  if(deviceConnected && transmit != true && !receiveImage && !wifiSetup && !serialWrite) {
    while(ATmegaSerial.available() > 0) {
      uint8_t byteFromSerial = ATmegaSerial.read();
      if(transmit != true && deviceConnected && !receiveImage) {
        uint8_t usart_buff[6];
        char* usart_out = "0xUT ";
       // usart_out += byteFromSerial;
        for(int i = 0; i < sizeof(usart_buff); i++) {
            usart_buff[i] = uint8_t(usart_out[i]);
        }
        usart_buff[5] = (uint8_t)byteFromSerial;
        pTxCharacteristic->setValue(usart_buff, sizeof(usart_buff));
        pTxCharacteristic->notify();
        delay(10);
      }
      if(serialCount > 10) {
        serialCount = 0;
        break;
      }
      serialCount ++;
    }
  }
  //ledcWrite(0, 10);
 // updateLED();
  if(wifiConnected == false && WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi Connected!" );
      wifiConnected = true;
    }

    if(WiFi.status() == WL_CONNECTED && (millis() - updateTimer) >= 60000) {
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
        digitalWrite(14, LOW);
        reset();
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        wifiStep = 0;
        wifiSetup = false;
        image = "";
        oldDeviceConnected = deviceConnected;
    }

}

// Methods
void initBLE() {
 // digitalWrite(12, HIGH);

  BLEDevice::init("Trynkit Husky");
  pServer = BLEDevice::createServer();
  // Set up callback function for whenever something is received.
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new ReceiveCallBack());
  pService->start();
  pServer->getAdvertising()->start();
}

void deinitBLE() {
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
}

void reset() {
  ESP.restart();
}

void fetchOTA() {
  deinitBLE();
  String url = "https://trynkit.us/api/fetch_ota/";
  url += wifi_settings.deviceID;
  url += "/";
  url += wifi_settings.deviceKey;
  http.begin(url.c_str(), HTTPS_CERT);
  int httpResp = http.GET();
  if(httpResp == 200) {
    String httpResp = http.getString();
    DynamicJsonBuffer JSONBuffer;                         //Memory pool
    JsonObject& parsed = JSONBuffer.parseObject(httpResp); //Parse message
    bool hasImage = parsed["has_update"];
    if(hasImage) {
      String codeImage = parsed["code"];
      image ="";
      image += codeImage;
      image.replace("\r", "");
      flashAtmega(&image);
      deleteOTA();
      receiveImage = false; 
    }
  }
  http.end();
  delay(100);
  reset();
}

void deleteOTA() {
  String url = "https://trynkit.us/api/delete_ota/";
  url += wifi_settings.deviceID;
  url += "/";
  url += wifi_settings.deviceKey;
  http.begin(url.c_str(), HTTPS_CERT);
  int httpResp = http.GET();
  http.end();
}


void disconnectWiFi() {
   wifiFlag = true;
   wifiConnected = false;
   WiFi.disconnect();
}

void reconnectWiFi() {
  wifiFlag = false;
  WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  WiFi.reconnect();
}

void MyServerCallbacks::onConnect(BLEServer* pServer) {
  deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
}

void writeSerial(String serialOut) {
  ATmegaSerial.flush();
  for(int i = 0; i < sizeof(serialOut); i++) {
    ATmegaSerial.write(serialOut[i]);
  }
}

// Handles BLE receives for images and flashing
void ReceiveCallBack::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxVal = pCharacteristic->getValue();
  String input = "";
  if(rxVal.length() > 0) {
    for(int i = 0; i < rxVal.length(); i++) {
      if(i % 2 == 0) {
        input += rxVal[i];
      }
    }
    if(serialWrite == true) {
      serialBuff += input;
        if(serialBuff.substring(serialBuff.length()-5, serialBuff.length()).equals("0x0UE")) {
        serialBuff = serialBuff.substring(0, serialBuff.length()-5);
        writeSerial(serialBuff);
        serialWrite = false;
        serialBuff = "";
        return;
      }
    }
    
    // BLE Flashing mode
    if(receiveImage == true) {
      image += input;
      if(image.substring(image.length()-5, image.length()).equals("0x0FC")) {
        transmitOut("0x0FC");
        image = image.substring(0, image.length()-5);
        flashAtmega(&image);
        Serial.println("Post BARF BLE");
        Serial.println(ESP.getFreeHeap());
        receiveImage = false;
        image = "";
        
        return;
      }
    }

    // Recieving WiFi Info from site
    if(wifiSetup == true) {
      wifi_settings.saved = false;
      wifiData += input;
       if(wifiData.substring(wifiData.length()-5, wifiData.length()).equals("0x0FC")) {
          wifiData = wifiData.substring(0, wifiData.length()-5);
          if(wifiStep == 0) {
            strcpy(wifi_settings.deviceName, wifiData.c_str()); 
            transmitOut("0x0DI");
          } else if(wifiStep == 1) {
            strcpy(wifi_settings.deviceID, wifiData.c_str()); 
            transmitOut("0x0DS");
          } else if(wifiStep == 2) {
            strcpy(wifi_settings.deviceKey, wifiData.c_str()); 
            transmitOut("0x0DD");
          } else if(wifiStep == 3) {
            strcpy(wifi_settings.ssid, wifiData.c_str()); 
            transmitOut("0x0DW");
          } else if(wifiStep == 4) {
            strcpy(wifi_settings.password, wifiData.c_str()); 
            transmitOut("0x00F");
            wifiStep = -1;
            wifiSetup = false;
          } else {
            wifiStep = -1;
            wifiSetup = false;
          }
          EEPROM.put(0, wifi_settings);
          EEPROM.commit();
          wifiStep ++;
          delay(200);
          wifiData = "";
        }
    }

    // Check for commands
    if(input.equals("0x0FA")) { // Flash via BLE
      transmitOut("0x0FB");
      receiveImage = true;
    }
   
    if(input.equals("0x0FB")) { // Update WiFi Settings
      transmitOut("0x0DN");
      wifiSetup = true;
    }

    if(input.equals("0x0UT")) {
      serialWrite = true;
      transmitOut("0x0UO");
    }
  }
}

void ReceiveCallBack::transmitOut(char* output) {
  //Serial.println(output);
  //memcpy(out_buff, output, sizeof(output));
  for(int i = 0; i < sizeof(out_buff); i++) {
    out_buff[i] = uint8_t(output[i]);
  }
  transmit = true;
}

void updateLED(void * pvParameters) {
  int dir = 0;
  int indexBlink = 0;
  //Serial.println("Asd");
  while(true) {
    //Serial.println(deviceConnected);
    vTaskDelay(20);
    if(deviceConnected) {
      ledcWrite(0, indexBlink);
      if(dir == 0) {
         indexBlink ++;
      } else {
        indexBlink --;
      }
  
      if(indexBlink >= 100) {
        dir = 1;
      } else if(indexBlink <= 0) {
        dir = 0;
      }
    } else {
        ledcWrite(0, 100);    
    }
  }
  
}
