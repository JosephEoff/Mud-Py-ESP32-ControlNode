//Example configuration for the Mud-Py flora ControlNode

const int     WATCHDOG_TIMEOUT_SECONDS = 120;
const int     RETRYPERIOD_SECONDS = 300;
const int     SENSORIDMESSAGETIMEOUT_SECONDS = 10;
const int     SENSORREADATTEMPTS = 4;
const int     MAXIMUMSENSORCOUNT = 30;

const char*   WLAN_SSID       = "SomeSSID";
const char*   WLAN_PASSWORD   = "SomePassword";

const char*   MQTT_HOST       = "192.168.0.2";
const int     MQTT_PORT       = 1883;
const char*   MQTT_CLIENTID   = "mud-py-node1"; //Also the hostname
const char*   MQTT_USERNAME   = "username";
const char*   MQTT_PASSWORD   = "password";
const String  MQTT_NODE_TOPIC = "mud-py-node"; 
const String  MQTT_BATTERY_TOPIC = "battery";
const String  MQTT_SENSOR_TOPIC = "mud-py-flora"; 
const String  MQTT_SENSORID_TOPIC = "sensorID";
const String  MQTT_SLEEP_TOPIC = "sleep";
const String  MQTT_DONE_TOPIC = "done";
const int     MQTT_RETRY_WAIT = 5000;
const int     MQTT_KEEPALIVE_SECONDS  = 3600;

const String  SENSOR_LIGHT = "light";
const String  SENSOR_BATTERY = "battery";
const String  SENSOR_MOISTURE = "moisture";
const String  SENSOR_TEMPERATURE = "temperature";
const String  SENSOR_CONDUCTIVITY = "conductivity";
const String  SENSOR_READTRYCOUNT = "readtries";
