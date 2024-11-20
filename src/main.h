#pragma once

#include "config.h"
#include "sensors.h"
#include "net.h"

uint32_t elapsedTime() {
    DateTime currentTime = rtc.now();
    TimeSpan elapsed = currentTime - startTime;
    
    return elapsed.totalseconds();
}

void initSystem() {
    pinMode(GPIO_NUM_2, OUTPUT);
    digitalWrite(GPIO_NUM_2, LOW);
    
    if (!rtc.begin()) {
        digitalWrite(GPIO_NUM_2, LOW);
        
        #ifdef DEBUG
        Serial.println("ERROR al inicializar RTC");
        #endif
        
        while (1);
    }

    sensors.begin();
    sensors.setResolution(11);
    
    ledcSetup(0, PWM_FREQ, PWM_RESOL);
    ledcAttachPin(RELAY_PIN, 0);
    
    Setpoint = DEFAULT_SETPOINT;
    controlPID.SetMode(AUTOMATIC);
    controlPID.SetOutputLimits(0, PWM_RANGE * PWM_K);
    controlPID.SetSampleTime(CONTROL_PERIOD);

    display.setBrightness(3); // 0 a 7

    startTime = rtc.now();
    pidActive = true;
    toggleDisplay = true;
    
    #ifdef DEBUG
    Serial.println("CONTROLADOR activo");
    #endif
}

void handleReporting() {
    const uint32_t duration_minutes = duration;

    if (toggleDisplay) {
        display.showNumberDec(Input, true);
    } else {
        uint8_t onWord[] = { 0x3F, 0x37, 0, 0 };
        uint8_t offWord[] = { 0x3F, 0x71, 0x71, 0 };
        pidActive ? display.setSegments(onWord): display.setSegments(offWord);
    }
    toggleDisplay = !toggleDisplay;

    if (mqttClient.connected()) {
        char payload[100];

        uint32_t elapsedMinutes = elapsedTime() / 60;
        uint32_t remainingMinutes = (DEFAULT_TIME_LIMIT > elapsedMinutes) ? (DEFAULT_TIME_LIMIT - elapsedMinutes) : 0;

        snprintf(payload, sizeof(payload), "{\"tem\":%.1f,\"set\":%.1f,\"dur\":%d,\"rem\":%d,\"sta\":\"%s\",\"p\":\"%.2f\",\"i\":\"%.2f\",\"d\":\"%.2f\"}", Input, Setpoint, duration_minutes, pidActive ? remainingMinutes : 0, pidActive ? "ON" : "OFF", Kp, Ki, Kd);
        mqttClient.publish(MQTT_REPORT_TOPIC, 0, false, payload);
    }
    
    #ifdef DEBUG
    Serial.printf("C %.1f T %.1f L %d S %s P %.2f I %.2f D %.2f\n", Input, Setpoint, duration_minutes, pidActive ? "ON" : "OFF", Kp, Ki, Kd);
    #endif
}

void stopHeating() {
    ledcWrite(0, 0);
    pidActive = false;
    targetReached = false;
    
    #ifdef DEBUG
    Serial.println("CONTROLADOR detenido");
    #endif
}
