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
#define MQTT_PASSWORD       "abc123"
#define MQTT_CLIENT_ID      "iduxnetepoxi2"
#define MQTT_ACK_TOPIC      "iduxnet/epoxi2/ack"
#define MQTT_REPORT_TOPIC   "iduxnet/epoxi2/temperature"
#define MQTT_COMMAND_TOPIC  "iduxnet/epoxi2/config"
#define MQTT_VERIFY_FREQ    30000

WiFiClient espClient;
PubSubClient mqttClient(espClient);
TaskHandle_t mqttTaskHandle;

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    #ifdef DEBUG
    Serial.println("WIFI conectado");
    #endif
}

void mqttTask(void* parameters);
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    digitalWrite(GPIO_NUM_2, HIGH);
    xTaskCreate(mqttTask, "mqttTask", 4 * 1024, NULL, 1, &mqttTaskHandle);

    #ifdef DEBUG
    Serial.print("IP ");
    Serial.println(WiFi.localIP());
    #endif
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    digitalWrite(GPIO_NUM_2, LOW);

    #ifdef DEBUG
    Serial.println("WIFI desconectado");
    #endif

    WiFi.reconnect();
}

void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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
            pidActive = true;
            startTime = rtc.now();
            control_timer = millis();
            report_timer = millis();
            targetReached = false;

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
}

void connectMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    
    while (!mqttClient.connected()) {
        if (mqttClient.connect("iduxnetepoxi2")) {
            digitalWrite(GPIO_NUM_2, HIGH);
            mqttClient.subscribe(MQTT_COMMAND_TOPIC);

            mqttClient.publish(MQTT_ACK_TOPIC, "conectado");
            
            #ifdef DEBUG
            Serial.println("Broker MQTT conectado");
            #endif
        } else {
            #ifdef DEBUG
            Serial.print("Broker MQTT desconectado");
            #endif
            
            delay(3000);
        }
    }
}

void mqttTask(void* parameters) {
    connectMQTT();

    for(;;) {
        while (true) {
            if (!mqttClient.connected()) connectMQTT();
            vTaskDelay(pdMS_TO_TICKS(MQTT_VERIFY_FREQ));
        }
    }
}