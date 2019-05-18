/*
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
#include "esp_ota_ops.h"
#include <esp_partition.h>
#include <Wire.h>

void setup() {
	Serial.begin(115200);
	// Restore WiFi settings if they exist
	EEPROM.begin(EEPROM_SIZE);
	EEPROM.get(0, wifi_settings);
	ATmegaSerial.begin(9600, SERIAL_8N1, 3, 1);

	ledcSetup(0, 5000, 8);
	ledcAttachPin(12, 0);

	// Set WiFi OTA Timer after everything else has been initialized
	updateTimer = millis();
	WiFi.mode(WIFI_STA);

	// LED update task
	TaskHandle_t LEDTask;
	xTaskCreatePinnedToCore(updateLED, "LEDTask", 10000, NULL, 1, &LEDTask, 0);

	//Init BLE
	initBLE();

	// Init WDT
	wdt = timerBegin(0, 80, true);
	timerAttachInterrupt(wdt, &watchdog_reset, true);
	timerAlarmWrite(wdt, watchdogReset * 1000, false);
	timerAlarmEnable(wdt)

	//Wire.begin();
	//getSerial();
}

void loop() {
	// Reset WDT
	timerWrite(wdt, 0);


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
		delay(250); // give the bluetooth stack the chance to get things ready
		pServer->startAdvertising(); // restart advertising
		oldDeviceConnected = deviceConnected;
		wifiStep = 0;
		wifiSetup = false;
		image = "";
		flashIdx = 0;
		reconnectWiFi();
		digitalWrite(14, LOW);
		reset();
	}
	// connecting
	if (deviceConnected && !oldDeviceConnected) {
		wifiStep = 0;
		wifiSetup = false;
		image = "";
		flashIdx = 0;
		oldDeviceConnected = deviceConnected;
	}

}

// Watchdog Interrupt
void IRAM_ATTR watchdog_reset() {
	esp_restart();
}

  if(deviceConnected && transmit != true && !receiveImage && !wifiSetup && !serialWrite) {
    uint8_t usart_buffer[64];
    char* usart_out = "0xUT ";
    for(int i = 0; i < sizeof(usart_out)+1; i++) {
        usart_buffer[i] = uint8_t(usart_out[i]);
    }
    int bufferCounter = 5;
    while(ATmegaSerial.available() > 0) {
      uint8_t serialByte = ATmegaSerial.read();
      usart_buffer[bufferCounter] = serialByte;
      if(serialByte != NULL)
        bufferCounter ++;
      delay(10);
      if(bufferCounter >= 64) {
        break;
      }
    }
    if(bufferCounter > 5) {
      uint8_t output_buffer[bufferCounter];
      for(int i = 0; i < bufferCounter; i++) {
        output_buffer[i] = usart_buffer[i];
      }
      delay(10);
      pTxCharacteristic->setValue(output_buffer, sizeof(output_buffer));
      pTxCharacteristic->notify();
      delay(10);
      bufferCounter = 5;
    }
  
  }
  //ledcWrite(0, 10);
 // updateLED();
  if(wifiConnected == false && WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
    }

    if(WiFi.status() == WL_CONNECTED && (millis() - updateTimer) >= 60000) {
      updateTimer = millis();
      fetchOTA();
    } else if((millis() - updateTimer) >= 60000) {
        if(wifi_settings.saved == false && setConnect == false) {
          setConnect = true;
          WiFi.begin(wifi_settings.ssid, wifi_settings.password);
        }
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
        flashIdx = 0;
        reconnectWiFi();
        digitalWrite(14, LOW);
        reset();
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        wifiStep = 0;
        wifiSetup = false;
        image = "";
        flashIdx = 0;
        oldDeviceConnected = deviceConnected;
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
      //image += input;
      
      for(int i = 0; i <input.length(); i++) {
        flash[flashIdx] = input[i]; 
        flashIdx++;
      }
      char img [5];
      for(int i = 5; i > 0; i--) {
        img[5-i] = char(flash[(flashIdx-i)]);  
      }
      transmitOut((char*)img);
      if(((String)img).equals("0x0FC")) {
        transmitOut("0x0FC");
        image = image.substring(0, image.length()-5);
        flashAtmega(flash);
        receiveImage = false;
        image = "";
        flashIdx = 0;
        
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

    if(input.equals("0xZU")) {
      const char* partName = "factory";
      PART = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, partName);
      esp_ota_set_boot_partition(PART);
      ESP.restart(); // restart ESP
    }
  }
}*/
