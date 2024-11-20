#include "config.h"
#include "main.h"
#include "net.h"
#include "sensors.h"

void setup() {
    #ifdef DEBUG
    Serial.begin(SERIAL_BAUDS);
    #endif

    initSystem();

    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(NET_RECONNECT_PERIOD), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(NET_RECONNECT_PERIOD), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    temperatureControlTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(CONTROL_PERIOD), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(controlTemperature));
    reportTimer = xTimerCreate("reportTimer", pdMS_TO_TICKS(REPORT_PERIOD), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(handleReporting));
    xTimerStop(temperatureControlTimer, 0);
    xTimerStop(reportTimer, 0);

    WiFi.onEvent(WiFiEvent);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);

    connectToWiFi();
}

void loop() {
    switch (currentState) {
        case IDLE: {
            if (xTimerIsTimerActive(temperatureControlTimer) == pdFALSE) xTimerStart(temperatureControlTimer, 0);
            break;
        }

        case INIT: {
            if (xTimerIsTimerActive(temperatureControlTimer) == pdFALSE) xTimerStart(temperatureControlTimer, 0);
            if (xTimerIsTimerActive(reportTimer) == pdFALSE) xTimerStart(reportTimer, 0);
            currentState = HEATING;
            break;
        }
        
        case HEATING: {
            if (pidActive && elapsedTime() >= duration * 60) {
                currentState = STOP;
            }
            break;
        }
        
        case STOP: {
            stopHeating();
            currentState = IDLE;
            break;
        }
    }
}
