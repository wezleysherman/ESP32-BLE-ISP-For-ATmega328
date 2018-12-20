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

void setup() {
  Serial.begin(115200);
  // Restore WiFi settings if they exist
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, wifi_settings);

  // Init BLE
  initBLE();
  
  // Set WiFi OTA Timer after everything else has been initialized
  updateTimer = millis();
  WiFi.mode(WIFI_STA);
  if(wifi_settings.saved == false) {
    WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  }
}

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

// Methods
void initBLE() {
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
      flashAtmega(image);
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
  Serial.println(wifiConnected);
  Serial.println(wifi_settings.ssid);
  Serial.println(wifi_settings.password);
  WiFi.begin(wifi_settings.ssid, wifi_settings.password);
  WiFi.reconnect();
  Serial.print("WiFi Reconnecting");
}

void MyServerCallbacks::onConnect(BLEServer* pServer) {
  deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
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

    // BLE Flashing mode
    if(receiveImage == true) {
      image += input;
      if(image.substring(image.length()-5, image.length()).equals("0x0FC")) {
        transmitOut("0x0FC");
        image = image.substring(0, image.length()-5);
        flashAtmega(image);
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
