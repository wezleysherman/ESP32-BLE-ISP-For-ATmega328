#ifndef FIRMWARE_H_INCLUDED
#define FIRMWARE_H_INCLUDED
// BLE UUID's for Trynkit Device
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID_RX "80c78362-e3f9-11e8-9f32-f2801f1b9fd1"
#define CHARACTERISTIC_UUID_TX "80c784ac-e3f9-11e8-9f32-f2801f1b9fd1"

// Pin Defines
#define EEPROM_SIZE 1000

// Global Variables
unsigned long updateTimer; // WiFi OTA Timer 

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
HTTPClient http;
HardwareSerial ATmegaSerial(1);

String trynkitURL = "https://trynkit.us";
String debugURL = "http://192.168.0.19:8000";

const esp_partition_t* PART;
byte flash[32000];
int flashIdx = 0;
bool wifiConnected = false;
bool setConnect = false;
// New
const int watchdog_timeout = 5000;
hw_timer_t *wdt = NULL;
hw_timer_t *deepsleep = NULL;
String recv_buffer = "";
unsigned char ble_state = 0;
bool old_device_connected = false;
bool device_connected = false;
uint8_t out_buff[7];
bool transmit = false;
bool flashing = false;
byte* flashPos;
bool receiving = false;
uint8_t usart_buffer[256];
int bufferCounter = 0;
String wifi_data = "";
unsigned char wifi_state = 0;
String serialNum = "";

void IRAM_ATTR deep_sleep();
void IRAM_ATTR watchdog_reset();

// Structs
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

// Method defines
void initBLE();
void reset();
void fetchOTA();

// Amazon's HTTPS certificate for Trynkit's backend -.. Expires?
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
#endif
