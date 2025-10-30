/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example, rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// LED Pin
const int ledPin = 2;  // Assuming the built-in LED is connected to pin 2

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue().c_str();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);

      Serial.println();
      Serial.println("*********");

      // Toggle LED based on received value
      if (rxValue == "!B813") {  //RIGHT ARROW PRESS
        turnRight();
      }
      else if (rxValue == "!B804") {  //RIGHT ARROW RELEASE
        stop();
      }
      else if (rxValue == "!B714") {  //LEFT ARROW PRESS
        turnLeft();
      }
      else if (rxValue == "!B705") {  //LEFT ARROW RELEASE
        stop();
      }
      else if (rxValue == "!B516") {  //UP ARROW PRESS
        goForward();
      }
      else if (rxValue == "!B507") {  //UP ARROW RELEASE
        stop();
      }
      else if (rxValue == "!B615") {  //DOWN ARROW PRESS
        goBackward();
      }
      else if (rxValue == "!B606") {  //DOWN ARROW RELEASE
        stop();
      }
    }
  }

  void turnRight() {
    rightMotor->setSpeed(550);
    leftMotor->setSpeed(550);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
  }

  void turnLeft() {
    rightMotor->setSpeed(550);
    leftMotor->setSpeed(550);
    rightMotor->run(BACKWARD);
    leftMotor->run(FORWARD);
  }

  void goForward() {
    rightMotor->setSpeed(550);
    leftMotor->setSpeed(550);
    rightMotor->run(BACKWARD);
    leftMotor->run(BACKWARD);
  }

  void goBackward() {
    rightMotor->setSpeed(550);
    leftMotor->setSpeed(550);
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
  }

  void stop() {
    rightMotor->setSpeed(0);
    leftMotor->setSpeed(0);
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);
  }
};

void setup() {
  Serial.begin(115200);
  AFMS.begin();
  pinMode(ledPin, OUTPUT);  // Set LED pin as output

  // Create the BLE Device
  BLEDevice::init("LittleYellowCar");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  if (deviceConnected) {
    pTxCharacteristic->setValue(&txValue, 1);
    pTxCharacteristic->notify();
    txValue++;
    delay(10);  // bluetooth stack will go into congestion, if too many packets are sent
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}
