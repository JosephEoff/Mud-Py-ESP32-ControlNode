//Example configuration for the Mud-Py flora ControlNode

const char*   WLAN_SSID       = "SomeSSID";
const char*   WLAN_PASSWORD   = "SomePassword";

const char*   MQTT_HOST       = "192.168.0.2";
const int     MQTT_PORT       = 1883;
const char*   MQTT_CLIENTID   = "mud-py-node1";
const char*   MQTT_USERNAME   = "username";
const char*   MQTT_PASSWORD   = "password";
const String  MQTT_BASE_TOPIC = "mud-py-flora"; 
const int     MQTT_RETRY_WAIT = 5000;
