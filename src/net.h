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
#define MQTT_REPORT_TOPIC   "iduxnet/epoxi2/temperature"
#define MQTT_COMMAND_TOPIC  "iduxnet/epoxi2/config"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWiFi() {
    #ifdef DEBUG
    Serial.print("Conectando a la red ");
    #endif
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        
        #ifdef DEBUG
        Serial.print(".");
        #endif
    }

    #ifdef DEBUG
    Serial.println(" OK!");
    #endif
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
    message.toUpperCase();
    String command = message.substring(0, 3);
    String value = message.substring(3);

    if (command == "SET") {
        Setpoint = value.toDouble();
    } else if (command == "STA") {
        value.toUpperCase();
        if (value == "ON") {
            startTime = rtc.now();
            currentState = HEATING;
        } else if (value == "OFF") {
            currentState = STOP;
        }
    } else if (command == "DUR") {
        duration = value.toInt();
    } else if (command == "PIP") {
        Kp = value.toDouble();
        controlPID.SetTunings(Kp, Ki, Kd);
    } else if (command == "PII") {
        Ki = value.toDouble();
        controlPID.SetTunings(Kp, Ki, Kd);
    } else if (command == "PID") {
        Kd = value.toDouble();
        controlPID.SetTunings(Kp, Ki, Kd);
    }
}

void connectMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    
    while (!mqttClient.connected()) {
        #ifdef DEBUG
        Serial.print("Conectando a servidor MQTT ...");
        #endif

        if (mqttClient.connect("iduxnetepoxi2")) {
            digitalWrite(GPIO_NUM_2, HIGH);
            mqttClient.subscribe(MQTT_COMMAND_TOPIC);
            
            #ifdef DEBUG
            Serial.println(" OK!");
            #endif
        } else {
            #ifdef DEBUG
            Serial.print(" ERROR: ");
            Serial.print(mqttClient.state());
            Serial.println(" (reintento en 5 segs)");
            #endif
            
            delay(5000);
        }
    }
}
