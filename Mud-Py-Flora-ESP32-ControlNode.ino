#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"

void setup() {
  float nodeBatteryVoltage = getNodeBatteryVoltage();
  connectToServer();
  subscribeToSensorIDMessage();
  sendActiveNotice(nodeBatteryVoltage);
  string() sensorIDs = readSensorIDListFromServer();
  disconnectFromServer();
  readSensorDataOverBluetooth();
  connectToServer();
  subscribeToSleepMessage();
  sendSensorUpdates();
  sendDone();
  int sleepTimeSeconds = readSleepPeriodFromMessage();
  disconnectFromServer();
  deepSleep(sleepTimeSeconds);  

}

void connectToServer(){
  connectWLAN();
  connectMQTT();
}

void disconnectFromServer(){
  disconnectMQTT();
  disconectWLAN();
}

void loop() {
  // put your main code here, to run repeatedly:

}
