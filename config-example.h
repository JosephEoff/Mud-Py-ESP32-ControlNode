//Example configuration for the Mud-Py flora ControlNode

const int     WATCHDOG_TIMEOUT_SECONDS = 60;
const int     RETRYPERIOD_SECONDS = 300;

const char*   WLAN_SSID       = "SomeSSID";
const char*   WLAN_PASSWORD   = "SomePassword";

const char*   MQTT_HOST       = "192.168.0.2";
const int     MQTT_PORT       = 1883;
const char*   MQTT_CLIENTID   = "mud-py-node1";
const char*   MQTT_USERNAME   = "username";
const char*   MQTT_PASSWORD   = "password";
const String  MQTT_NODE_TOPIC = "mud-py-node"; 
const String  MQTT_BATTERY_TOPIC = "battery";
const String  MQTT_SENSOR_TOPIC = "mud-py-flora"; 
const int     MQTT_RETRY_WAIT = 5000;
