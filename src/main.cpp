#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <TM1637Display.h>

#define DEBUG
#define ONE_WIRE_BUS        4
#define RELAY_PIN           32
#define SEG7_DIO_PIN        10
#define SEG7_CLOCK_PIN      13
#define PWM_RANGE           255
#define PWM_K               .75 //.075 // ajustar s/ potencia calefactor
#define PWM_FREQ            5000
#define PWM_RESOL           8
#define SERIAL_BAUDS        115200
#define DEFAULT_TIME_LIMIT  30 // mins
#define DEFAULT_SETPOINT    35.0 // °C
#define CONTROL_PERIOD      500 // ms
#define REPORT_PERIOD       3000 // ms

double Setpoint, Input, Output;
/**
 * Kp: alto, respuesta rápida pero oscilación; bajo, respuesta lenta.
 * Ki: alto, ajuste rápido de errores pero oscilación, bajo, arrastre de errores.
 * Kd: alto, más estabilidad pero respuesta lenta, bajo, mejor respuesta con oscilaciones.
 */
bool pidActive;
// double Kp = 5.0, Ki = 0.05, Kd = 0.5;
double Kp = 4.0, Ki = 0.025, Kd = 0.01;
uint32_t control_timer, report_timer, off_timer;
uint32_t duration = DEFAULT_TIME_LIMIT * 60 * 1000;

enum States { IDLE, INIT, HEATING, STOP };
States currentState = INIT;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
PID controlPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
TM1637Display display(SEG7_CLOCK_PIN, SEG7_DIO_PIN);

void readSensors(uint8_t index) {
    sensors.requestTemperatures();
    Input = sensors.getTempCByIndex(index);
    if ((int)Input == -127) Input = 0;
}

void initSystem() {
    pidActive = true;
    off_timer = millis();
    report_timer = millis();
    
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
    const uint32_t duration_minutes = duration / 60000;
    
    // displayTemperature(Input);
    display.showNumberDec(Input, true);
    
    #ifdef DEBUG
    Serial.printf("C %.1f T %.1f L %d S %s\n", Input, Setpoint, duration_minutes, pidActive ? "ON" : "OFF");
    #endif
}

void stopHeating() {
    ledcWrite(0, 0);
    pidActive = false;
    
    #ifdef DEBUG
    Serial.println("CONTROLADOR detenido");
    #endif
}

void setup() {
    #ifdef DEBUG
    Serial.begin(SERIAL_BAUDS);    
    #endif

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
            
            if (pidActive && millis() - off_timer >= duration) {
                currentState = STOP;
            }
            break;
        
        case STOP:
            stopHeating();
            currentState = IDLE;
            break;
    }
}
