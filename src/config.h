#pragma once

#include <Arduino.h>
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
#define DEFAULT_SETPOINT    30.0 // °C
#define CONTROL_PERIOD      500 // ms
#define REPORT_PERIOD       10000 // ms

/**
 * Kp: alto, respuesta rápida pero oscilación; bajo, respuesta lenta.
 * Ki: alto, ajuste rápido de errores pero oscilación, bajo, arrastre de errores.
 * Kd: alto, más estabilidad pero respuesta lenta, bajo, mejor respuesta con oscilaciones.
 */
bool pidActive, targetReached, toggleDisplay;
double Setpoint, Input, Output;
double Kp = 3.0, Ki = 0.1, Kd = 0.01;
uint32_t control_timer, report_timer, off_timer;
uint32_t duration = DEFAULT_TIME_LIMIT * 60 * 1000;

enum States { IDLE, INIT, HEATING, STOP };
States currentState = INIT;

DateTime startTime;
PID controlPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
TM1637Display display(SEG7_CLOCK_PIN, SEG7_DIO_PIN);
RTC_DS3231 rtc;
