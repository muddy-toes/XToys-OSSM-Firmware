#include "BLEManager.h"
#include "config.h"

namespace BLEManager {

  bool deviceConnected = false;
  QueueHandle_t messageQueue = nullptr;

  void (*msgReceivedCallback)(String);

  BLEUUID CONTROL_UUID = BLEUUID(CONTROL_CHARACTERISTIC_UUID);

  BLEServer *pServer;
  BLEService *pService;
  BLECharacteristic *controlCharacteristic;
  BLEService *infoService;
  BLECharacteristic *softwareAPIVersionCharacteristic;
  BLECharacteristic *softwareFirmwareVersionCharacteristic;

  void MessageCallbacks::onWrite(BLECharacteristic *characteristic) {
    std::string cppStr = characteristic->getValue();
    if (characteristic->getUUID().equals(CONTROL_UUID)) {
      if (messageQueue != nullptr && cppStr.length() < MAX_MESSAGE_LENGTH) {
        char buffer[MAX_MESSAGE_LENGTH];
        strncpy(buffer, cppStr.c_str(), MAX_MESSAGE_LENGTH - 1);
        buffer[MAX_MESSAGE_LENGTH - 1] = '\0';
        xQueueSend(messageQueue, buffer, 0);  // Non-blocking, drop if full
      }
    }
  }

  // Client connected to OSSM over BLE
  void ServerCallbacks::onConnect(BLEServer* pServer) {
    BLEManager::deviceConnected = true;
  }

  void ServerCallbacks::onDisconnect(BLEServer* pServer) {
    BLEManager::deviceConnected = false;
    pServer->startAdvertising();
    // TODO: stop
  }

  void sendNotification (String message) {
    if (!deviceConnected) return;
    // Use the data-parameter overload to send our exact bytes atomically.
    // The setValue()+notify() pattern is racy: XToys writes volume updates to
    // this same characteristic every ~100ms, and those writes overwrite the
    // characteristic value. If a write lands between setValue and notify, the
    // notification sends XToys' data instead of ours.
    controlCharacteristic->notify(
      (const uint8_t*)message.c_str(), message.length(), true);
  }

  void processQueue() {
    if (messageQueue == nullptr || msgReceivedCallback == nullptr) return;
    char buffer[MAX_MESSAGE_LENGTH];
    while (xQueueReceive(messageQueue, buffer, 0) == pdTRUE) {
      msgReceivedCallback(String(buffer));
    }
  }

  void setup (String bleName, void (*msgReceivedCallback)(String)) {

    BLEManager::msgReceivedCallback = msgReceivedCallback;

    // Create message queue for decoupling BLE callbacks from processing
    messageQueue = xQueueCreate(MESSAGE_QUEUE_SIZE, MAX_MESSAGE_LENGTH);

    //###################
    // Initiate Bluetooth
    //###################
    NimBLEDevice::init(bleName.c_str());
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    infoService = pServer->createService(NimBLEUUID((uint16_t) 0x180a));
    BLE2904* softwareVersionDescriptor = new BLE2904();
    softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
    softwareVersionDescriptor->setNamespace(1);
    softwareVersionDescriptor->setUnit(0x27ad);

    softwareAPIVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a28, NIMBLE_PROPERTY::READ);
    softwareAPIVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
    softwareAPIVersionCharacteristic->setValue(API_VERSION);

    softwareFirmwareVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a26, NIMBLE_PROPERTY::READ);
    softwareFirmwareVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
    softwareFirmwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
    infoService->start();
    
    pService = pServer->createService(SERVICE_UUID);
    controlCharacteristic = pService->createCharacteristic(
                                          CONTROL_UUID,
                                          NIMBLE_PROPERTY::READ |
                                          NIMBLE_PROPERTY::WRITE |
                                          NIMBLE_PROPERTY::NOTIFY
                                        );
    controlCharacteristic->setValue("");
    controlCharacteristic->setCallbacks(new MessageCallbacks());

    pService->start();
    NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // min connection interval (7.5ms)
    pAdvertising->setMaxPreferred(0x12); // max connection interval (22.5ms)
    NimBLEDevice::startAdvertising();
  }

  bool isConnected() {
    return deviceConnected;
  }
}