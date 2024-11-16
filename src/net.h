#pragma once

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define WIFI_SSID           "CARFI"
#define WIFI_PASSWORD       "carfi2024"
// #define MQTT_BROKER         "f79f3acc160242ddb508cb6bc61b51e2.s1.eu.hivemq.cloud"
// #define MQTT_PORT           8883
// #define MQTT_BROKER         "broker.hivemq.com"
// #define MQTT_PORT           1883
#define MQTT_BROKER         "broker.emqx.io"
#define MQTT_PORT           1883
#define MQTT_USER           "cperren"
#define MQTT_PASSWORD       "DieBill666**"
#define MQTT_CLIENT_ID      "iduxnetepoxi2"
#define MQTT_TOPIC          "iduxnet/epoxi2/temperature"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWiFi() {
    Serial.print("Conectando a la red ");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println(" OK!");
}

void connectMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    
    while (!mqttClient.connected()) {
        Serial.print("Conectando a servidor MQTT ...");
        if (mqttClient.connect("iduxnetepoxi2")) {
            digitalWrite(GPIO_NUM_2, HIGH);
            Serial.println(" OK!");
        } else {
            Serial.print(" ERROR: ");
            Serial.print(mqttClient.state());
            Serial.println(" (reintento en 5 segs)");
            delay(5000);
        }
    }
}
