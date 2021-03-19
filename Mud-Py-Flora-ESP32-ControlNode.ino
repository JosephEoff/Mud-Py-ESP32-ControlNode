#include <esp_task_wdt.h>
#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"
#include "FloraDecoder.h"

WiFiClient espWLAN;
PubSubClient MQTTclient(espWLAN);

const int BatteryMonitor_ADC_pin = 36;

static BLEUUID FloraServiceUUID("00001204-0000-1000-8000-00805f9b34fb");
static BLEUUID FloraVersionAndBatteryUUID("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID FloraSensorDataUUID("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID FloraWriteModeUUID("00001a00-0000-1000-8000-00805f9b34fb");
String topicSensorID = "sensorID";
String topicNode = "";
int64_t timeOfLastMessage;
int sleepTimeSeconds = -1;
bool sentDoneMessage = false;


void setup() {
  Serial.begin(115200);
  delay(1000);

  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P7);

  setupWatchdog();
  topicNode = MQTT_NODE_TOPIC + "/" + WiFi.macAddress() +"/"; 
  
  float nodeBatteryVoltage = getNodeBatteryVoltage();
  connectToServer();
  subscribeToSensorIDMessage();
  subscribeToSleepMessage();
  sendActiveNotice(nodeBatteryVoltage);
  timeOfLastMessage = esp_timer_get_time();
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  timeOfLastMessage = esp_timer_get_time();
  String topicString = topic;
  Serial.println("MQTTcallback topic:" + topicString);
  payload[length] = '\0';
  char* payloadString = (char*)payload;
  handleSensorTopic(topicString, payloadString);
  handleSleepTopic(topicString, payloadString);
  timeOfLastMessage = esp_timer_get_time();
}

void handleSensorTopic(String topic, char* payload){
  esp_task_wdt_reset();
  Serial.println("handleSensorTopic topic: " + topicNode + MQTT_SENSORID_TOPIC);
  if (topic == topicNode + MQTT_SENSORID_TOPIC){
    Serial.println("handleSensorTopic topic matched: " + topic);
    BLEAddress sensorID(payload);
    
    Serial.print("handleSensorTopic BLEAddress: ");
    Serial.println(sensorID.toString().c_str());
    processSensor(sensorID);
  }
  timeOfLastMessage = esp_timer_get_time();
  esp_task_wdt_reset();
}

void processSensor(BLEAddress SensorID){
  int counter = 0;
  Serial.println("processSensor ");
  while (counter<SENSORREADATTEMPTS){
    counter += 1;
    esp_task_wdt_reset();
    if (readSensorAndReportViaMQTT(SensorID)){
      break;
    }
    esp_task_wdt_reset();
  }
   esp_task_wdt_reset();
}

bool readSensorAndReportViaMQTT(BLEAddress SensorID){
  Serial.println("readSensorAndReportViaMQTT ");
  BLEClient* sensorClient = getSensorClient(SensorID);
  
  esp_task_wdt_reset();
  
  if (sensorClient == nullptr) {
    return false;
  }

  BLERemoteService* sensorService = getSensorService(sensorClient);
  
  esp_task_wdt_reset();

  if (sensorService == nullptr) {
    sensorClient->disconnect();
    return false;
  }

  if (!switchSensorToDataMode(sensorService)){
    sensorClient->disconnect();
    return false;
  }
  esp_task_wdt_reset();

  bool sentSensorData = readSensorDataAndSendViaMQTT(sensorService, SensorID);
  esp_task_wdt_reset();
  readSensorBatteryLevelAndSendViaMQTT(sensorService, SensorID);
  esp_task_wdt_reset();
  sensorClient->disconnect();
  esp_task_wdt_reset();

  //Don't care if the sensor battery data made it, just if the sensor data was read and sent.
  return sentSensorData;
}

BLEClient* getSensorClient(BLEAddress sensorAddress) {
  Serial.println("getSensorClient ");
  esp_task_wdt_reset();
  BLEClient* sensorClient = BLEDevice::createClient();
  esp_task_wdt_reset();

  if (!sensorClient->connect(sensorAddress)) {
    Serial.println("getSensorClient failed: sensorClient->connect(sensorAddress) ");
    return nullptr;
  }

  return sensorClient;
}

BLERemoteService* getSensorService(BLEClient* sensorClient) {
  Serial.println("getSensorService ");
  BLERemoteService* sensorService = nullptr;

  try {
    esp_task_wdt_reset();
    sensorService = sensorClient->getService(FloraServiceUUID);
    esp_task_wdt_reset();
  }
  catch (...) {

  }

  return sensorService;
}

bool switchSensorToDataMode(BLERemoteService* sensorService) {
  Serial.println("switchSensorToDataMode");
  BLERemoteCharacteristic* sensorCharacteristic;
  
  sensorCharacteristic = nullptr;
  try {
    esp_task_wdt_reset();
    sensorCharacteristic = sensorService->getCharacteristic(FloraWriteModeUUID);
    esp_task_wdt_reset();
  }
  catch (...) {

  }
  if (sensorCharacteristic == nullptr) {
    return false;
  }

  // send command
  uint8_t buf[2] = {0xA0, 0x1F};
  sensorCharacteristic->writeValue(buf, 2, true);

  delay(500);
  return true;
}

bool readSensorDataAndSendViaMQTT(BLERemoteService* sensorService, BLEAddress SensorID){
  Serial.println("readSensorDataAndSendViaMQTT");
  esp_task_wdt_reset();
  BLERemoteCharacteristic* sensorDataCharacteristic = nullptr;
  try {
    sensorDataCharacteristic = sensorService->getCharacteristic(FloraSensorDataUUID);
    esp_task_wdt_reset();
  }
  catch (...) {
    // something went wrong
  }
  if (sensorDataCharacteristic == nullptr) {
    return false;
  }
  
  esp_task_wdt_reset();
  
  //Read data from sensor
  std::string sensorData;
  try{
    esp_task_wdt_reset();
    sensorData = sensorDataCharacteristic->readValue();
    esp_task_wdt_reset();
  }
  catch (...) {
    return false;
  }
  
  esp_task_wdt_reset();
  
  FloraDecoder::DecodeSensorData(sensorData);

  if (!FloraDecoder::decodeOK_Sensor){
    return false;
  }

  bool sentOK = sendSensorDataViaMQTT(SensorID);
  return sentOK;
}

bool sendSensorDataViaMQTT(BLEAddress SensorID){
  Serial.println("sendSensorDataViaMQTT");
  try{
    publishMQTTMessage(MQTT_SENSOR_TOPIC,String(SensorID.toString().c_str()), SENSOR_LIGHT, String(FloraDecoder::sensor_Light));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,String(SensorID.toString().c_str()), SENSOR_MOISTURE, String(FloraDecoder::sensor_Moisture));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,String(SensorID.toString().c_str()), SENSOR_TEMPERATURE, String(FloraDecoder::sensor_Temperature));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,String(SensorID.toString().c_str()), SENSOR_CONDUCTIVITY, String(FloraDecoder::sensor_Conductivity));    
    esp_task_wdt_reset();
  }
  catch(...){
    return false;
  }
  return true;
}

void readSensorBatteryLevelAndSendViaMQTT(BLERemoteService* sensorService,  BLEAddress SensorID){
  Serial.println("sendSensorDataViaMQTT");
  BLERemoteCharacteristic* sensorBatteryCharacteristic = nullptr;
  try {
    sensorBatteryCharacteristic = sensorService->getCharacteristic(FloraVersionAndBatteryUUID);
  }
  catch (...) {
    // something went wrong
  }
  if (sensorBatteryCharacteristic == nullptr) {
    return;
  }
  
  esp_task_wdt_reset();
  
  //Read data from sensor
  std::string sensorData;
  try{
    sensorData = sensorBatteryCharacteristic->readValue();
  }
  catch (...) {
    return;
  }
  
  esp_task_wdt_reset();
  
  FloraDecoder::DecodeBattery(sensorData);

  if (FloraDecoder::decodeOK_Sensor){
    publishMQTTMessage(MQTT_SENSOR_TOPIC,String(SensorID.toString().c_str()), SENSOR_BATTERY, String(FloraDecoder::sensor_Battery));
  }
}

void handleSleepTopic(String topic, char*  payload){
  esp_task_wdt_reset();
  Serial.println("handleSleepTopic topic:" + topic);
  Serial.println("handleSleepTopic topic to match:" + topicNode + MQTT_SLEEP_TOPIC);
  if (topic == topicNode + MQTT_SLEEP_TOPIC){
    String sleepString = String(payload);
    int sleepTime = sleepString.toInt();
    Serial.println("handleSleepTopic sleeptime: " + String(sleepTime));
    disconnectFromServer();
    takeANap(sleepTime);
  }
  timeOfLastMessage = esp_timer_get_time();
  esp_task_wdt_reset();
}

void subscribeToSensorIDMessage(){
  String topic = MQTT_NODE_TOPIC +"/" + WiFi.macAddress() + "/" + MQTT_SENSORID_TOPIC;
  Serial.println("subscribeToSensorIDMessage topic:" + topic);
  MQTTclient.subscribe(topic.c_str(), 0);
}

void subscribeToSleepMessage(){
  String topic = MQTT_NODE_TOPIC +"/" + WiFi.macAddress() + "/" + MQTT_SLEEP_TOPIC;
  Serial.println("subscribeToSleepMessage topic:" + topic);
  MQTTclient.subscribe(topic.c_str(), 0);
}

void setupWatchdog(){
   esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, false);
   esp_task_wdt_add(NULL);
}

void esp_task_wdt_isr_user_handler(void){
  takeANap(RETRYPERIOD_SECONDS);
}

void takeANap(int sleepTimeSeconds){
  Serial.println("takeANap sleepTimeSeconds: " + String(sleepTimeSeconds));
  esp_sleep_enable_timer_wakeup((uint64_t)sleepTimeSeconds * 1000000);
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
    delay(100);
  }
}

void connectMQTT(){
  MQTTclient.setServer(MQTT_HOST, MQTT_PORT);
  MQTTclient.setCallback(MQTTcallback);
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
  publishMQTTMessage(MQTT_NODE_TOPIC, WiFi.macAddress(), MQTT_BATTERY_TOPIC, String(batteryVoltage));
}

void sendDoneMessage(){
  publishMQTTMessage(MQTT_NODE_TOPIC, WiFi.macAddress(), MQTT_DONE_TOPIC, "1");
  sentDoneMessage = true;
  timeOfLastMessage = esp_timer_get_time();
  esp_task_wdt_reset();
}

void publishMQTTMessage(String baseTopic, String ID, String topic, String content){
  String completeTopic = baseTopic+ "/" + ID + "/" +topic;
  Serial.println("publishMQTTMessage topic: " + completeTopic + " content:" + content);
  char buffer[64];
  
  esp_task_wdt_reset();

  snprintf(buffer, 64, content.c_str());
  MQTTclient.publish(completeTopic.c_str(), buffer); 
  
  esp_task_wdt_reset();
}

void loop() {
  MQTTclient.loop();
  if (esp_timer_get_time() - timeOfLastMessage > SENSORIDMESSAGETIMEOUT_SECONDS*1000000){
    if (sentDoneMessage){
        takeANap(RETRYPERIOD_SECONDS);
    }
    else{
      sendDoneMessage();
    }
  }
}
