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

    connectWiFi();
    connectMQTT();
}

void loop() {
    switch (currentState) {
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
            
            if (pidActive && limitTimeReached()) {
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

    mqttClient.loop();
}
