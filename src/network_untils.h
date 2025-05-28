#ifndef NETWORK_UNTILS_H
#define NETWORK_UNTILS_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>

// WiFi
extern const char *ssid;
extern const char *password;

// MQTT
extern const char *mqtt_server;
extern WiFiClient espClient;
extern PubSubClient client;

// MQTT Callback
extern float setDegree;

void setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void TaskMQTT(void *pvParameters);
void TaskPublish(void *pvParameters);

#endif