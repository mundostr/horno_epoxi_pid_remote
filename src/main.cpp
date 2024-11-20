#include "config.h"
#include "main.h"
#include "net.h"
#include "sensors.h"

void setup() {
    #ifdef DEBUG
    Serial.begin(SERIAL_BAUDS);
    #endif

    pinMode(GPIO_NUM_2, OUTPUT);
    digitalWrite(GPIO_NUM_2, LOW);

    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(3000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(3000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
        
    WiFi.onEvent(WiFiEvent);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);

    connectToWiFi();
}

void loop() {
    /* switch (currentState) {
        case IDLE: {
            if (millis() - report_timer >= REPORT_PERIOD) {
                readSensors(0);
                handleReporting();
                report_timer = millis();
            }
            break;
        }

        case INIT: {
            initSystem();
            currentState = HEATING;
            break;
        }
        
        case HEATING: {
            if (millis() - control_timer >= CONTROL_PERIOD) {
                handleHeating();
                control_timer = millis();
            }

            if (millis() - report_timer >= REPORT_PERIOD) {
                handleReporting();
                report_timer = millis();
            }
            
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
    } */
}
