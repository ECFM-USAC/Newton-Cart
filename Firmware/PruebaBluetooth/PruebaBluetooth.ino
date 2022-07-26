#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertisedDevice.h> // iDK if necessary 
#include <BLEScan.h>

BLECharacteristic* Distance;
BLECharacteristic* Force;
bool deviceConnected = false;
float distanceValue = 0;
float forceValue = 0;

#define SERVICE_UUID            "0000180D-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_DISTANCE_UUID  "00002713-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_FORCE_UUID  "00002714-0000-1000-8000-00805F9B34FB"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    deviceConnected = true;
    };

  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);

  //Create the BLE Device
  BLEDevice::init("ESP32PET");

  //Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  //Create the BLE Service

  BLEService *pService = pServer->createService(SERVICE_UUID);

  //Create a BLE Characteristic
  Distance = pService->createCharacteristic(
                      CHARACTERISTIC_DISTANCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                      );
  //Create a BLE Characteristic
  Force = pService->createCharacteristic(
                      CHARACTERISTIC_FORCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                      );
                                           
  //BLE2902 needed to notify
  Distance->addDescriptor(new BLE2902());
  Force->addDescriptor(new BLE2902());
  
  //Start the service
  pService->start();

  //Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");

}

void loop() {
  if (deviceConnected){
    distanceValue = random(10,20);
    forceValue = random(50,60);
    
    //Conversion of txValue
    char distanceString[8];
    char forceString[8];
    dtostrf(distanceValue,1,2,distanceString);
    dtostrf(forceValue,1,2,forceString);
    
    //Setting the value to the characteristic
    Distance->setValue(distanceString);
    Force->setValue(forceString);
    
    //Notifying the connected client
    Distance->notify();
    Force->notify();
    Serial.println("Sent value: " + String(distanceString));
    Serial.println("Sent value2: " + String(forceString));
    Serial.println();
    delay(2000);
    }

}
