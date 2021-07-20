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
char sendBuffer[64];
std::string sensorData;

int sensorIDsCount = 0;
BLEAddress *sensorIDs[MAXIMUMSENSORCOUNT];


void setup() {
  Serial.begin(115200);
  delay(1000);

  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  int sensorIDsCount = 0;

  setupWatchdog();
  topicNode = MQTT_NODE_TOPIC + "/" + WiFi.macAddress() +"/"; 
  
  float nodeBatteryVoltage = getNodeBatteryVoltage();
  esp_task_wdt_reset();
  connectToServer();
  esp_task_wdt_reset();
  subscribeToSensorIDMessage();
  esp_task_wdt_reset();
  subscribeToSleepMessage();
  esp_task_wdt_reset();
  long rssi = WiFi.RSSI();
  esp_task_wdt_reset();
  sendActiveNotice(nodeBatteryVoltage, rssi);
  esp_task_wdt_reset();
  timeOfLastMessage = esp_timer_get_time();
  doProcessingLoop();
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  String messageTopic = topic;
  Serial.println("MQTTcallback topic:" + messageTopic);
  payload[length] = '\0';
  char* payloadString = (char*)payload;
  handleSensorTopic(messageTopic, payloadString);
  handleSleepTopic(messageTopic, payloadString);
  timeOfLastMessage = esp_timer_get_time();
}

void handleSensorTopic(String topic, char* payload){
  esp_task_wdt_reset();
  Serial.println("handleSensorTopic topic: " + topicNode + MQTT_SENSORID_TOPIC);
  if (topic == topicNode + MQTT_SENSORID_TOPIC){
    Serial.println("handleSensorTopic topic matched: " + topic);

    if (sensorIDsCount<MAXIMUMSENSORCOUNT){
         sensorIDs[sensorIDsCount] = new BLEAddress(payload);
         Serial.print("handleSensorTopic BLEAddress: ");
         Serial.println(sensorIDs[sensorIDsCount]->toString().c_str());
         sensorIDsCount += 1;
         
    }
  }
  
  timeOfLastMessage = esp_timer_get_time();
  esp_task_wdt_reset();
}

void processSensor(BLEAddress SensorID){
  int counter = 0;
  Serial.println("processSensor ");
  String ID = String(SensorID.toString().c_str());
  while (counter<SENSORREADATTEMPTS){
    counter += 1;
    esp_task_wdt_reset();
    if (readSensorAndReportViaMQTT(SensorID, ID)){
      break;
    }
    esp_task_wdt_reset();

  }
 
   publishMQTTMessage(MQTT_SENSOR_TOPIC, ID, SENSOR_READTRYCOUNT, String(counter));
   esp_task_wdt_reset();
}

bool readSensorAndReportViaMQTT(BLEAddress SensorID, String ID){
  Serial.println("readSensorAndReportViaMQTT ");
  BLEClient* sensorClient = getSensorClient(SensorID);
  
  esp_task_wdt_reset();
  
  if (sensorClient == nullptr) {
    return false;
  }

  BLERemoteService* sensorService = getSensorService(sensorClient);
  
  esp_task_wdt_reset();

  if (sensorService == nullptr) {
    disconnectSensorClient(sensorClient);
    return false;
  }

  if (!switchSensorToDataMode(sensorService)){
    disconnectSensorClient(sensorClient);
    return false;
  }
  esp_task_wdt_reset();

  bool sentSensorData = readSensorDataAndSendViaMQTT(sensorService, ID );
  esp_task_wdt_reset();
  readSensorBatteryLevelAndSendViaMQTT(sensorService, ID);
  esp_task_wdt_reset();
  if (sentSensorData){
    int rssi = sensorClient->getRssi();
    sendSensorRSSIViaMQTT(ID,rssi);
  }
  disconnectSensorClient(sensorClient);
  esp_task_wdt_reset();

  //Don't care if the sensor battery data made it, just if the sensor data was read and sent.
  return sentSensorData;
}
void sendSensorRSSIViaMQTT(String SensorID, int rssi){
  publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_RSSI, String(rssi));
  esp_task_wdt_reset();
}

void disconnectSensorClient(BLEClient* sensorClient){
    try {
      sensorClient->disconnect();
    }
    catch (...) {
      
    }
    
    delay(1000);
    
    try {
       delete sensorClient;
    }
    catch (...) {      
    }
}


BLEClient* getSensorClient(BLEAddress sensorAddress) {
  Serial.println("getSensorClient ");
  BLEClient* sensorClient;

  esp_task_wdt_reset();
  try { 
    sensorClient = BLEDevice::createClient();  
  }
  catch (...) {
     esp_task_wdt_reset();
     return nullptr;
  }
  
  esp_task_wdt_reset();

  if (sensorClient==nullptr){
    return nullptr;
  }

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
    return nullptr;
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

bool readSensorDataAndSendViaMQTT(BLERemoteService* sensorService, String SensorID){
  Serial.println("readSensorDataAndSendViaMQTT");
  esp_task_wdt_reset();
  BLERemoteCharacteristic* sensorDataCharacteristic = nullptr;
  try {
    sensorDataCharacteristic = sensorService->getCharacteristic(FloraSensorDataUUID);
    esp_task_wdt_reset();
  }
  catch (...) {
    
  }
  if (sensorDataCharacteristic == nullptr) {
    return false;
  }
  
  esp_task_wdt_reset();
  
  //Read data from sensor

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

bool sendSensorDataViaMQTT(String SensorID){
  Serial.println("sendSensorDataViaMQTT");
  try{
    publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_LIGHT, String(FloraDecoder::sensor_Light));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_MOISTURE, String(FloraDecoder::sensor_Moisture));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_TEMPERATURE, String(FloraDecoder::sensor_Temperature));
    esp_task_wdt_reset();
    publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_CONDUCTIVITY, String(FloraDecoder::sensor_Conductivity));    
    esp_task_wdt_reset();
  }
  catch(...){
    return false;
  }
  return true;
}

void readSensorBatteryLevelAndSendViaMQTT(BLERemoteService* sensorService,  String SensorID){
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

  if (FloraDecoder::decodeOK_Battery){
    publishMQTTMessage(MQTT_SENSOR_TOPIC,SensorID, SENSOR_BATTERY, String(FloraDecoder::sensor_Battery));
  }
}

void handleSleepTopic(String topic, char* payload){
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
  WiFi.setHostname(MQTT_CLIENTID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
}

void connectMQTT(){
  MQTTclient.setServer(MQTT_HOST, MQTT_PORT);
  MQTTclient.setCallback(MQTTcallback);
  MQTTclient.setKeepAlive(MQTT_KEEPALIVE_SECONDS);
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

void sendActiveNotice(float batteryVoltage, long rssi){
  publishMQTTMessage(MQTT_NODE_TOPIC, WiFi.macAddress(), MQTT_NODE_RSSI_TOPIC, String(rssi));
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

  
  esp_task_wdt_reset();
  reconnectMQTTIfNeeded();
  esp_task_wdt_reset();
  snprintf(sendBuffer, 64, content.c_str());
  Serial.println("Buffer ready.");
  MQTTclient.publish(completeTopic.c_str(), sendBuffer); 
  Serial.println("Message published");
  esp_task_wdt_reset();
}

void reconnectMQTTIfNeeded(){
    if (!MQTTclient.connected()) {
      esp_task_wdt_reset();
      Serial.println("Reconnect to MQTT server.");
      connectMQTT();
      esp_task_wdt_reset();
  }
}

void restartIfConnectionLost(){
  if (!MQTTclient.connected()) {
    takeANap(RETRYPERIOD_SECONDS);
  }
}

void doProcessingLoop(){
  while(true){
    processLoop();
  }  
}

void processLoop(){
  esp_task_wdt_reset();
  //reconnectMQTTIfNeeded();
  restartIfConnectionLost();
  esp_task_wdt_reset();
  
  MQTTclient.loop();  
  
  if (esp_timer_get_time() - timeOfLastMessage > SENSORIDMESSAGETIMEOUT_SECONDS*1000000){
    readAndSendDataFromAllSensors();
    if (sentDoneMessage){
        takeANap(RETRYPERIOD_SECONDS);
    }
    else{
      sendDoneMessage();
    }
  }
}

void readAndSendDataFromAllSensors(){
  int counter = 0;
  while (counter<sensorIDsCount){
     processSensor(*sensorIDs[counter]);
     counter += 1;
  }
 
}


void loop() {
 //Should never get here.  If something goes wrong, take a nap and recover.
 takeANap(RETRYPERIOD_SECONDS);
}
