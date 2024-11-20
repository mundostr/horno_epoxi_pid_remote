#pragma once

#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID               "CARFI"
#define WIFI_PASSWORD           "carfi2024"
// #define MQTT_BROKER          "f79f3acc160242ddb508cb6bc61b51e2.s1.eu.hivemq.cloud"
// #define MQTT_BROKER          "broker.hivemq.com"
#define MQTT_BROKER             "broker.emqx.io"
#define MQTT_PORT               1883
#define MQTT_USER               "cperren"
#define MQTT_PASSWORD           "abc123"
#define MQTT_CLIENT_ID          "iduxnetepoxi2"
#define MQTT_ACK_TOPIC          "iduxnet/epoxi2/ack"
#define MQTT_REPORT_TOPIC       "iduxnet/epoxi2/temperature"
#define MQTT_COMMAND_TOPIC      "iduxnet/epoxi2/config"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer, wifiReconnectTimer;

void connectToWiFi() {
    #ifdef DEBUG
    Serial.println("Conectando WiFi");
    #endif
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println("Conectando MQTT");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP: {
            connectToMqtt();

            #ifdef DEBUG
            Serial.println("WiFi conectado");
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            #endif
            
            break;
        }
        
        case SYSTEM_EVENT_STA_DISCONNECTED: {
            xTimerStop(mqttReconnectTimer, 0);
            xTimerStart(wifiReconnectTimer, 0);

            #ifdef DEBUG
            Serial.println("WiFi desconectado");
            #endif
            
            break;
        }

        default:
            break;
    }
}

void onMqttConnect(bool sessionPresent) {
    digitalWrite(GPIO_NUM_2, HIGH);
    mqttClient.subscribe(MQTT_COMMAND_TOPIC, 0);
    mqttClient.publish(MQTT_ACK_TOPIC, 0, false, "conectado");

    #ifdef DEBUG
    Serial.println("MQTT conectado");
    #endif
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    digitalWrite(GPIO_NUM_2, LOW);
    if (WiFi.isConnected()) xTimerStart(mqttReconnectTimer, 0);

    #ifdef DEBUG
    Serial.println("MQTT desconectado");
    #endif
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    String message;
    for (unsigned int i = 0; i < len; i++) message += (char)payload[i];
    message.toUpperCase();
    String command = message.substring(0, 3);
    String value = message.substring(3);

    if (command == "SET") {
        Setpoint = value.toDouble();
    } else if (command == "STA") {
        value.toUpperCase();
        if (value == "ON") {
            pidActive = true;
            targetReached = false;
            startTime = rtc.now();

            currentState = HEATING;
        } else if (value == "OFF") {
            currentState = STOP;
        }
    } else if (command == "DUR") {
        duration = value.toInt();
        startTime = rtc.now();
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

    #ifdef DEBUG
    Serial.print(command);
    Serial.println(value);
    #endif
}
