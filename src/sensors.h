#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>

#include "config.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void readSensors(uint8_t index) {
    sensors.requestTemperatures();
    Input = sensors.getTempCByIndex(index);
    if ((int)Input == -127) Input = 0;
    if ((int)Input >= DEFAULT_SETPOINT && !targetReached) {
        targetReached = true;
        startTime = rtc.now();
    }
}

void controlTemperature(TimerHandle_t xTimer) {
    readSensors(0);

    if (pidActive) {
        controlPID.Compute();
        if (Input > (Setpoint * 0.85)) Output *= 0.5;
        ledcWrite(0, (int)Output);
        
        #ifdef DEBUG
        Serial.printf("PWM: %d\n", (int)Output);
        #endif
    }
}
