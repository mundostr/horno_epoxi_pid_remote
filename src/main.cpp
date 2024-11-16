#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <RTClib.h>

#define DEBUG

#define ONE_WIRE_BUS        4
#define RELAY_PIN           32
#define SEG7_DIO_PIN        10
#define SEG7_CLOCK_PIN      13

#define PWM_RANGE           255
#define PWM_K               .5 // ajustar s/ potencia calefactor
#define PWM_FREQ            5000
#define PWM_RESOL           8
#define SERIAL_BAUDS        115200
#define DEFAULT_TIME_LIMIT  30 // mins -> 48 hs = 2880, 5 hs = 300
#define DEFAULT_SETPOINT    50.0 // °C
#define CONTROL_PERIOD      500 // ms
#define REPORT_PERIOD       10000 // ms

#include "net.h"

/**
 * Kp: alto, respuesta rápida pero oscilación; bajo, respuesta lenta.
 * Ki: alto, ajuste rápido de errores pero oscilación, bajo, arrastre de errores.
 * Kd: alto, más estabilidad pero respuesta lenta, bajo, mejor respuesta con oscilaciones.
 */
bool pidActive, targetReached, toggleDisplay;
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 0.02, Kd = 0.02;
uint32_t control_timer, report_timer, off_timer;
uint32_t duration = DEFAULT_TIME_LIMIT * 60 * 1000;

enum States { IDLE, INIT, HEATING, STOP };
States currentState = INIT;

DateTime startTime;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
PID controlPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
TM1637Display display(SEG7_CLOCK_PIN, SEG7_DIO_PIN);
RTC_DS3231 rtc;

void readSensors(uint8_t index) {
    sensors.requestTemperatures();
    Input = sensors.getTempCByIndex(index);
    if ((int)Input == -127) Input = 0;
    if ((int)Input >= DEFAULT_SETPOINT && !targetReached) {
        targetReached = true;
        startTime = rtc.now();
    }
}

void initSystem() {
    if (!rtc.begin()) {
        digitalWrite(GPIO_NUM_2, LOW);
        
        #ifdef DEBUG
        Serial.println("ERROR al inicializar RTC");
        #endif
        
        while (1);
    }

    startTime = rtc.now();
    pidActive = true;
    toggleDisplay = true;
    
    #ifdef DEBUG
    Serial.println("CONTROLADOR activo");
    #endif
}

void handleHeating() {
    if (pidActive) {
        readSensors(0);
        controlPID.Compute();
        ledcWrite(0, (int)Output);
        
        #ifdef DEBUG
        Serial.printf("PWM: %d\n", (int)Output);
        #endif
    }
}

void handleReporting() {
    const uint32_t duration_minutes = DEFAULT_TIME_LIMIT;

    if (toggleDisplay) {
        display.showNumberDec(Input, true);
    } else {
        uint8_t onWord[] = { 0x3F, 0x37, 0, 0 };
        uint8_t offWord[] = { 0x3F, 0x71, 0x71, 0 };
        pidActive ?  display.setSegments(onWord):  display.setSegments(offWord);
    }
    toggleDisplay = !toggleDisplay;

    if (mqttClient.connected()) {
        char payload[50];
        snprintf(payload, sizeof(payload), "{\"tem\":%.1f,\"set\":%.1f,\"dur\":%d,\"sta\":\"%s\"}", Input, Setpoint, duration_minutes, pidActive ? "ON" : "OFF");
        mqttClient.publish(MQTT_TOPIC, payload);
    }
    
    #ifdef DEBUG
    Serial.printf("C %.1f T %.1f L %d S %s\n", Input, Setpoint, duration_minutes, pidActive ? "ON" : "OFF");
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

bool limitTimeReached() {
    DateTime currentTime = rtc.now();
    TimeSpan elapsed = currentTime - startTime;
    
    return elapsed.totalseconds() >= DEFAULT_TIME_LIMIT * 60;
}

void setup() {
    #ifdef DEBUG
    Serial.begin(SERIAL_BAUDS);    
    #endif

    pinMode(GPIO_NUM_2, OUTPUT);
    digitalWrite(GPIO_NUM_2, LOW);

    connectWiFi();
    connectMQTT();

    sensors.begin();
    sensors.setResolution(11);
    ledcSetup(0, PWM_FREQ, PWM_RESOL);
    ledcAttachPin(RELAY_PIN, 0);
    
    Setpoint = DEFAULT_SETPOINT;
    controlPID.SetMode(AUTOMATIC);
    controlPID.SetOutputLimits(0, PWM_RANGE * PWM_K);

    display.setBrightness(3); // 0 a 7
}

void loop() {
    switch (currentState) {
        case IDLE:
            if (millis() - report_timer >= REPORT_PERIOD) {
                readSensors(0);
                handleReporting();
                report_timer = millis();
            }
            break;

        case INIT:
            initSystem();
            currentState = HEATING;
            break;
        
        case HEATING:
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
        
        case STOP:
            stopHeating();
            currentState = IDLE;
            break;
    }

    mqttClient.loop();
}
