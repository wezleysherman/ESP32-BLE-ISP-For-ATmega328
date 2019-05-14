#ifndef BLE_H_INCLUDED
#define BLE_H_INCLUDED
class MyServerCallbacks: public BLEServerCallbacks {
  public:
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

class ReceiveCallBack: public BLECharacteristicCallbacks {
  public:
    void onWrite(BLECharacteristic *pCharacteristic);
    void transmitOut(char* output);
};
#endif
