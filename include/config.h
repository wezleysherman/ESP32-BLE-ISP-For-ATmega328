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

typedef struct husky_settings {
	int firmware_version;
	unsigned char low_power_timeout; // minutes -- active if greater than 0
	unsigned char low_power_check; // minutes -- active if greater than 0
  char* device_id;
} HUSKY;

struct husky_settings config_defaults = {
	.firmware_version = 1,
	.low_power_timeout = 0,
	.low_power_check = 0,
  .device_id = "0000ffe0-0000-1000-8000-00805f9b34fb"
};

WIFI wifi_settings;
HUSKY husky_settings;