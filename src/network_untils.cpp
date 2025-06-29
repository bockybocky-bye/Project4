#include "network_untils.h"
#include "mpu6050.h"

// WiFi
const char *ssid = "bighair1";
const char *password = "0909641456";

// MQTT
const char *mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// Node-RED control input
float setDegree = 0.0;

// -------------------- WiFi Setup --------------------
void setup_wifi() {
  Serial.println("\n📡 Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (++attempts > 20) {
      Serial.println("❌ WiFi failed");
      return;
    }
  }
  Serial.println("\n✅ WiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

// -------------------- MQTT Reconnect --------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔄 MQTT connecting...");
    if (client.connect("ESP32_MPU6050")) {
      Serial.println("✅ connected");
      client.subscribe("esp32/set");
    } else {
      Serial.print("❌ failed, rc="); Serial.println(client.state());
      delay(2000);
    }
  }
}

// -------------------- MQTT Callback --------------------
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "esp32/set") {
    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (!error && doc.containsKey("set")) {
      setDegree = doc["set"];
      Serial.print("🎯 SetDegree ใหม่: ");
      Serial.println(setDegree);
    }
  }
  else if (String(topic) == "esp32/cmd") {
    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (!error && doc.containsKey("cmd")) {
        String cmd = doc["cmd"];
        if (cmd == "reset") {
            Serial.println("🔄 ESP32 กำลังรีสตาร์ท...");
            delay(500);
            ESP.restart();
        }
    }
  }
}

// -------------------- Task: MQTT Loop --------------------
void TaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// -------------------- Task: Publish Sensor --------------------
void TaskPublish(void *pvParameters) {
  for (;;) {
    char payload[100];
    sprintf(payload, "{\"roll\":%.2f,\"pitch\":%.2f,\"set\":%.2f}", KalmanAngleRoll, KalmanAnglePitch, setDegree);
    client.publish("esp32/degree", payload);
    Serial.println(payload);
    vTaskDelay(95 / portTICK_PERIOD_MS);
  }
}