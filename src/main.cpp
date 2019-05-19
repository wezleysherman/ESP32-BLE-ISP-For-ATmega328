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
#include "ECC508.h"
#include "esp32-hal-cpu.h"


void setup() {
	Serial.begin(115200);
	// Restore WiFi settings if they exist
	EEPROM.begin(EEPROM_SIZE);
	EEPROM.get(0, wifi_settings);
	ATmegaSerial.begin(9600, SERIAL_8N1, 3, 1);
	//Serial.println(wifi_settings.ssid);
	//Serial.println(wifi_settings.deviceKey);
	//Serial.println(wifi_settings.deviceID);
	//Serial.println(wifi_settings.saved);
	//Serial.println(wifi_settings.password);
//
	ledcSetup(0, 5000, 8);
	ledcAttachPin(12, 0);

	// Set WiFi OTA Timer after everything else has been initialized
	updateTimer = millis();
	WiFi.mode(WIFI_STA);
	TaskHandle_t LEDTask;
	xTaskCreatePinnedToCore(updateLED, "LEDTask", 10000, NULL, 1, &LEDTask, 0);
	//Init BLE
	initBLE();

	// Init WDT
	wdt = timerBegin(0, 80, true);
	timerAttachInterrupt(wdt, &watchdog_reset, true);
	timerAlarmWrite(wdt, watchdog_timeout * 1000, false);
	timerAlarmEnable(wdt);

	Serial.println(getCpuFrequencyMhz());
	setCpuFrequencyMhz(80);
	Serial.println(getCpuFrequencyMhz());
	//
	//
	Wire.begin();
	serialNum = getSerial();
	//Serial.println("Rip: ");
	//Serial.println(serialNum);
}

int serialCount = 0;

void loop() {
	// Reset WDT
	timerWrite(wdt, 0);

	// Update BLE
	if(device_connected) { // Connected
		disconnectWiFi(); // If Wifi is enabled -- disable it.
		if(!flashing && !receiving) {
			// Check recv
			process_ble_recv();
		}
		// Check transmit
		if(transmit == true) {
			delay(20); // Give it a second to disable
			pTxCharacteristic->setValue(out_buff, 5);
			pTxCharacteristic->notify();
			delay(10);
			uint8_t empty[6];
			transmit = false;
			memcpy(out_buff, empty, sizeof(empty));
		}
	} 

	if(!device_connected && old_device_connected) { // Disconnecting
		Serial.println("Disconnecting");
    	esp_restart(); // Reset ESP32 to clear RAM -- on BLE disconnect
	} 

	if(device_connected && !old_device_connected) { // Connecting
		Serial.println("Connecting");
		setCpuFrequencyMhz(240);
		old_device_connected = device_connected;
	}

	if(flashing) {
		flashPos = flashAtmega(flashPos);
		if(flashPos == nullptr) {
			Serial.println("Done");
			flashIdx = 0;
			memset(flash, 0xFF, sizeof flash);
			flashing = false;
		}
	}

	// Update USART
	if(device_connected && !flashing && ble_state == 0 && !receiving && !transmit && (ATmegaSerial.available() > 0 || bufferCounter > 0)) {

		if(bufferCounter < 5) {
			char* usart_out = "0xUT ";
			for(int i = 0; i < sizeof(usart_out)+1; i++) {
				usart_buffer[i] = uint8_t(usart_out[i]);
				bufferCounter ++;
			}
		}
		
		if (ATmegaSerial.available() > 0) {
			uint8_t serialByte = ATmegaSerial.read();
			usart_buffer[bufferCounter] = serialByte;

			if(serialByte != NULL)
				bufferCounter ++;
		}

		if (bufferCounter > 255 || (ATmegaSerial.available() == 0 && bufferCounter > 5)) {
			uint8_t output_buffer[bufferCounter];
			for(int i = 0; i < bufferCounter; i++) {
				output_buffer[i] = usart_buffer[i];
			}
			pTxCharacteristic->setValue(output_buffer, sizeof(output_buffer));
			pTxCharacteristic->notify();
			bufferCounter = 0;
			memset(output_buffer, 0xFF, sizeof output_buffer);
		}
	}

	// Update WiFi
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
}

// BLE FSM?
void process_ble_recv() {
	switch(ble_state) {
		case 0:
			if(recv_buffer.equals("0x0FA")) {
				transmitOut("0x0FB");
				ble_state = 1;
			} else if(recv_buffer.equals("0x0FB")) {
				transmitOut("0x0DN");
				wifi_state = 0;
				ble_state = 2;
			} else if(recv_buffer.equals("0x0UT")) {
				transmitOut("0x0UO");
				ble_state = 3;
			} else if(recv_buffer.equals("0xZU")) {
				const char* partName = "factory";
				PART = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, partName);
				esp_ota_set_boot_partition(PART);
				ESP.restart(); // restart ESP
			} else if(recv_buffer.equals("0x0FW")) {
				transmitOut("0x0DN");
				ble_state = 4;
			}
			break;
		case 1:			// Flash
			if(recv_buffer.substring(recv_buffer.length()-5, recv_buffer.length()).equals("0x0FC")) {
				for(int i = 0; i < recv_buffer.length(); i++) {
					flash[flashIdx] = recv_buffer[i];
					flashIdx++;
				}
				String output = "";
				
				flashPos = flash;

				transmitOut("0x0FC");
				flashing = true;
				ble_state = 0;
			} else {
				for(int i = 0; i < recv_buffer.length(); i++) {
					flash[flashIdx] = recv_buffer[i];
					flashIdx++;
				}
				if(flashIdx > 32000) {
					flashIdx = 0;
					memset(flash, 0xFF, sizeof flash);
					ble_state = 0;
					transmitOut("0x0F1");
				}
			}
			break;
		case 2:			// WiFi Scan
			{
			WiFi.disconnect();
			String networks_json = "\"";
			networks_json += serialNum;
			networks_json += "\":{";
			int networks = WiFi.scanNetworks();
			for(unsigned char i = 0; i < networks; i++) {
				networks_json += "\"";
				networks_json += WiFi.SSID(i);
				networks_json += "\":[";
				networks_json += WiFi.RSSI(i);
				networks_json += ", ";
				networks_json += WiFi.encryptionType(i);
				networks_json += "]";
				if(i != networks-1)
					networks_json += ",";
			}
			networks_json += "}";
			int index = 0;
			int buffer_size = 255;
			while(index < networks_json.length()) {
				if((networks_json.length()-index) < 255) {
					buffer_size = (networks_json.length() - index);
				}
				Serial.println(buffer_size);
				uint8_t networks_buffer[buffer_size];
				for(int i = 0; i < buffer_size; i++) {
					networks_buffer[i] = networks_json[i+index];
				}
				pTxCharacteristic->setValue(networks_buffer, sizeof(networks_buffer));
				pTxCharacteristic->notify();
				memset(networks_buffer, 0xFF, sizeof networks_buffer);
				index += buffer_size;
				delay(100);
			}
			transmitOut("0x0DZ");
			ble_state = 0;
			}
			break;
		case 3:			// Serial Write
			serialBuff += recv_buffer;
			if(serialBuff.substring(serialBuff.length()-5, serialBuff.length()).equals("0x0UE")) {
			  serialBuff = serialBuff.substring(0, serialBuff.length()-5);
			  serialBuff += '\r';
			  writeSerial(serialBuff);
			  serialBuff = "";
			  ble_state = 0;
			}
			break;
		case 4:
			{
			wifi_settings.saved = false;
			wifi_data += recv_buffer;
			if(wifi_data.substring(wifi_data.length()-5, wifi_data.length()).equals("0x0FC")) {
				wifi_data = wifi_data.substring(0, wifi_data.length()-5);
				if(wifi_state  == 0) {
					strcpy(wifi_settings.deviceName, wifi_data.c_str()); 
					transmitOut("0x0DI");
				} else if(wifi_state == 1) {
					strcpy(wifi_settings.deviceID, wifi_data.c_str()); 
					transmitOut("0x0DS");
				} else if(wifi_state == 2) {
					strcpy(wifi_settings.deviceKey, wifi_data.c_str()); 
					transmitOut("0x0DD");
				} else if(wifi_state == 3) {
					strcpy(wifi_settings.ssid, wifi_data.c_str()); 
					transmitOut("0x0DW");
				} else if(wifi_state == 4) {
					strcpy(wifi_settings.password, wifi_data.c_str()); 
					transmitOut("0x00F");
					ble_state = 0;
					wifi_state = 0;
				} else {
					ble_state = 0;
					wifi_state = 0;
				}
				EEPROM.put(0, wifi_settings);
				EEPROM.commit();
				wifi_state ++;
				delay(200);
				wifi_data = "";
			}
		}
		break;
	}
	recv_buffer = "";
}

// Watchdog Interrupt
void IRAM_ATTR watchdog_reset() {
  esp_restart();
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
	String url = debugURL + "/api/fetch_ota/";
	url += wifi_settings.deviceID;
	url += "/";
	url += wifi_settings.deviceKey;
	http.begin(url.c_str());//, HTTPS_CERT);
	int httpResp = http.GET();
	if(httpResp == 200) {
		String httpResp = http.getString();
		DynamicJsonBuffer JSONBuffer;                         //Memory pool
		JsonObject& parsed = JSONBuffer.parseObject(httpResp); //Parse message
		bool hasImage = parsed["has_update"];
		if(hasImage) {
			String parsedCode = parsed["code"];
			parsedCode.replace("\r", "");
			for(int i = 0; i < parsedCode.length(); i++) {
				flash[i] = parsedCode[i];
			}
			Serial.println(parsedCode);
			flashing = true;
			flashPos = flash;
			while(flashing) {
				flashPos = flashAtmega(flashPos);
				if(flashPos == nullptr) {
					Serial.println("Done");
					flashIdx = 0;
					memset(flash, 0xFF, sizeof flash);
					flashing = false;
				}
			}
			deleteOTA();
		}
	}
	http.end();
	delay(100);
	reset();
}

void deleteOTA() {
	String url = debugURL + "/api/delete_ota/";
	url += wifi_settings.deviceID;
	url += "/";
	url += wifi_settings.deviceKey;
	http.begin(url.c_str());//, HTTPS_CERT);
	int httpResp = http.GET();
	http.end();
}

void disconnectWiFi() {
	wifiConnected = false;
	WiFi.disconnect();
}

void reconnectWiFi() {
	WiFi.begin(wifi_settings.ssid, wifi_settings.password);
	WiFi.reconnect();
}

void MyServerCallbacks::onConnect(BLEServer* pServer) {
	device_connected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
	device_connected = false;
}

void writeSerial(String serialOut) {
	ATmegaSerial.flush();
	for(int i = 0; i < sizeof(serialOut); i++) {
		ATmegaSerial.write(serialOut[i]);
	}
}

// Handles BLE receives for images and flashing
void ReceiveCallBack::onWrite(BLECharacteristic *pCharacteristic) {
	receiving = true;
    std::string rxVal = pCharacteristic->getValue();
	if(rxVal.length() > 0) {
		for(int i = 0; i < rxVal.length(); i++) {
			if(i % 2 == 0) {
				recv_buffer += rxVal[i];
			}
		}
	}
	receiving = false;
}

void transmitOut(char* output) {
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
    if(device_connected) {
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