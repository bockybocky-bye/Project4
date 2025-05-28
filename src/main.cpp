// ESP32 MPU6050 + MQTT + FreeRTOS Integration

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h> 

// WiFi
const char *ssid = "bighair1";
const char *password = "0909641456";

// MQTT
const char *mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// MPU6050
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

// Node-RED control input
float setDegree = 0.0;

// PID Parameters
float Kp = 1.5, Ki = 0.3, Kd = 0.1;
float error = 0, integral = 0, derivative = 0, prev_error = 0;
unsigned long lastPID = 0;
Servo motor;

// -------------------- Kalman Filter --------------------
void kalman_1d(float &state_estimate, float &uncertainty, float rate_input, float measurement) {
  float dt = 0.004;             // Sampling time [s]
  float Q = dt * dt * 4.0 * 4.0;     // Process noise covariance (gyro)
  float R = 3.0 * 3.0;                // Measurement noise covariance (acc)

  // Prediction step
  state_estimate += dt * rate_input;  // x̂⁻ = x̂ + dt·u
  uncertainty += Q;                   // P⁻ = P + Q

  // Correction step
  float kalman_gain = uncertainty / (uncertainty + R);       // K = P⁻ / (P⁻ + R)
  state_estimate += kalman_gain * (measurement - state_estimate);  // x̂ = x̂⁻ + K(z - x̂⁻)
  uncertainty *= (1.0 - kalman_gain);                        // P = (1 - K)P⁻

  Kalman1DOutput[0] = state_estimate;
  Kalman1DOutput[1] = uncertainty;
}

// -------------------- อ่านค่า MPU6050 --------------------
void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
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
}

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

// -------------------- Task: Read MPU6050 --------------------
void TaskMPU6050(void *pvParameters) {
  for (;;) {
    gyro_signals();
    RateRoll  -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw   -= RateCalibrationYaw;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    vTaskDelay(4 / portTICK_PERIOD_MS);
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

// -------------------- Task: PID Control --------------------
void TaskPIDControl(void *pvParameters) {
  for (;;) {
    unsigned long now = millis();
    float dt = (now - lastPID) / 1000.0;
    lastPID = now;

    error = setDegree - KalmanAngleRoll;
    integral += error * dt;
    derivative = (error - prev_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    int pwm = constrain(map(output, -45, 45, 1000, 2000), 1000, 2000);
    motor.writeMicroseconds(pwm);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  for (int i = 0; i < 2000; i++) {
    gyro_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }
  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;

  // ⭐ Set ค่าเริ่มต้นให้ Kalman จากค่ามุมที่วัดได้ขณะนิ่ง
  KalmanAngleRoll = AngleRoll;
  KalmanAnglePitch = AnglePitch;
  motor.attach(32); // กำหนดขา PWM ควบคุม ESC/Servo

  xTaskCreatePinnedToCore(TaskMPU6050, "MPU6050", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskPublish, "Publish", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskPIDControl, "PID", 4096, NULL, 1, NULL, 1);
}

// -------------------- Loop --------------------
void loop() {

}
