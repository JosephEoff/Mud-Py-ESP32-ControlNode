#include <esp_task_wdt.h>
#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"

WiFiClient espWLAN;
PubSubClient MQTTclient(espWLAN);

const int BatteryMonitor_ADC_pin = 36;

void setup() {
  setupWatchdog();
  float nodeBatteryVoltage = getNodeBatteryVoltage();
  connectToServer();
  //subscribeToSensorIDMessage();
  sendActiveNotice(nodeBatteryVoltage);
  //string() sensorIDs = readSensorIDListFromServer();
  disconnectFromServer();
  //readSensorDataOverBluetooth();
  //connectToServer();
  //subscribeToSleepMessage();
  //sendSensorUpdates();
  //sendDone();
  //int sleepTimeSeconds = readSleepPeriodFromMessage();
  //disconnectFromServer();
  //takeANap(sleepTimeSeconds);  
  takeANap(30);

}
void setupWatchdog(){
   esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, false);
   esp_task_wdt_add(NULL);
}

void esp_task_wdt_isr_user_handler(void){
  takeANap(WATCHDOG_TIMEOUT_SECONDS);
}

void takeANap(int sleepTimeSeconds){
  esp_sleep_enable_timer_wakeup(sleepTimeSeconds * 1000000);
  esp_deep_sleep_start();
}

float getNodeBatteryVoltage(){
  int ADC_VALUE = analogRead(BatteryMonitor_ADC_pin);
  float voltage_value = (((float)ADC_VALUE) * 3.3 ) / (4095.0) * 2;
  return voltage_value;
}

void connectToServer(){
  connectWLAN();
  esp_task_wdt_reset();
  connectMQTT();
  esp_task_wdt_reset();
}

void connectWLAN(){
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void connectMQTT(){
  MQTTclient.setServer(MQTT_HOST, MQTT_PORT);
  while (!MQTTclient.connected()) {
    if (!MQTTclient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      delay(MQTT_RETRY_WAIT);
    }
  }
}
  
void disconnectFromServer(){
  disconnectMQTT();
  esp_task_wdt_reset();
  disconnectWLAN();
  esp_task_wdt_reset();
}

void disconnectWLAN(){
    WiFi.disconnect(true);
}

void disconnectMQTT(){
    MQTTclient.disconnect();
}

void sendActiveNotice(float batteryVoltage){
  publishMQTTMessage(MQTT_NODE_TOPIC,WiFi.macAddress(), MQTT_BATTERY_TOPIC, String(batteryVoltage));
}

void publishMQTTMessage(String baseTopic, String ID, String topic, String content){
  char buffer[64];
  
  esp_task_wdt_reset();

  snprintf(buffer, 64, content.c_str());
  MQTTclient.publish((baseTopic+ "/" + ID + "/" +topic).c_str(), buffer); 
  
  esp_task_wdt_reset();
}

void loop() {
  // put your main code here, to run repeatedly:

}
