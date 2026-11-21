#pragma once
#include <PCF8574.h>
#include <math.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "I2C.hpp"

#include <FS.h>
#include <SPIFFS.h>

PCF8574 pcf8574(0x20);

#define SPEED_PIN 16
const int pwmFrequencySpeed = 1000; // 1 kHz
const int pwmResolution = 8;         // 8 bit

#define ENCODER_PIN 34

// Używamy zmiennej globalnej do zliczania impulsów z enkodera
volatile long indents = 0;
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelayMicros = 5000;

volatile int motorSpeed = 160;
volatile const int maxSpeed = 255;  // Maksymalna wartość prędkości (0-255)
int pwmChannelSpeed = 9;
bool busy_motors = false;
bool busy_forward = false;
bool busy_backward = false;

// Kierunek jazdy dla UI. Zwykły wskaźnik na literał zamiast String:
// String nie jest bezpieczny wątkowo (zapis z auto-jazdy/BLE, odczyt z serwera WWW).
const char* volatile direction = "Stop";

// Pozycja robota (x, y, kąt), droga z enkodera i żyroskop: patrz Odometry.hpp.

// Deklaracje funkcji sterujących
typedef void (*DirectionFunction)();
void forward();
void backward();
void turn_left();
void turn_right();
void stop_driving();

typedef void (*TurnFunction)();

// Funkcja przerwania enkodera z debouncingiem.
// IRAM_ATTR jest obowiązkowe na ESP32 - bez tego crash, gdy cache flasha jest wyłączony
// (zapis SPIFFS, operacje WiFi).
void IRAM_ATTR encoderInterrupt() {
  unsigned long interruptTime = micros();
  if (interruptTime - lastInterruptTime > debounceDelayMicros) {
    indents++;  // Zliczamy impuls, jeśli minął czas debouncingu
    lastInterruptTime = interruptTime;
  }
}

// Stan ostatnio ustawiony na mostkach (PCF8574). Dzięki temu powtórne wywołanie
// forward()/stop_driving() w każdej iteracji pętli nie generuje 8 transakcji I2C.
enum MotionState { MOTION_UNKNOWN, MOTION_STOP, MOTION_FORWARD, MOTION_BACKWARD, MOTION_LEFT, MOTION_RIGHT };
volatile MotionState currentMotion = MOTION_UNKNOWN;
unsigned long lastMotionWrite = 0;
const unsigned long MOTION_REFRESH_MS = 500; // co tyle odświeżamy stan, nawet jeśli bez zmian

// Zwraca true, gdy PCF już ma ten stan (zapisany niedawno) - wtedy nie trzeba pisać ponownie.
static bool motionUpToDate(MotionState s) {
  unsigned long now = millis();
  if (currentMotion == s && (now - lastMotionWrite) < MOTION_REFRESH_MS) return true;
  currentMotion = s;
  lastMotionWrite = now;
  return false;
}

// Żyroskop. Odczytuje go wyłącznie odometria (zadanie czujników), patrz Odometry.hpp.
MPU6050 mpu(Wire);

void pcf8574_init() {
  for (int i = 0; i < 8; i++) {
    pcf8574.pinMode(i, OUTPUT);
  }
  pcf8574.begin();

//////////////////////////////////
///////  DRIVING TEST  ///////////
//////////////////////////////////
//drive(forward,100,140);
//delay(1000);
//turn(turn_right,90,255);
//delay(1000);
//turn(turn_right,70,255);
//delay(1000);
//drive(forward,200,140);
//////////////////////////////////
}

// Każda funkcja kierunku ustawia WSZYSTKIE flagi busy_* jawnie
// (wcześniej obrót po jeździe do przodu zostawiał busy_forward = true).
void forward() {
  busy_motors = true;
  busy_forward = true;
  busy_backward = false;
  direction = "Do przodu";
  if (motionUpToDate(MOTION_FORWARD)) return;
  I2CLock lock(300);   // zapis do PCF pod blokadą I2C; po 300 ms piszemy mimo wszystko (stop ma priorytet)
  //Wheel 1
  pcf8574.digitalWrite(0, LOW);
  pcf8574.digitalWrite(1, HIGH);
  //Wheel 2
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW);
  //Wheel 3
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH);
  //Wheel 4
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW);
}

void backward() {
  busy_motors = true;
  busy_forward = false;
  busy_backward = true;
  direction = "Do tyłu";
  if (motionUpToDate(MOTION_BACKWARD)) return;
  I2CLock lock(300);   // zapis do PCF pod blokadą I2C; po 300 ms piszemy mimo wszystko (stop ma priorytet)
  //Wheel 1
  pcf8574.digitalWrite(0, HIGH);
  pcf8574.digitalWrite(1, LOW);
  //Wheel 2
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH);
  //Wheel 3
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW);
  //Wheel 4
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH);
}

void turn_left() {
  busy_motors = true;
  busy_forward = false;
  busy_backward = false;
  direction = "W lewo";
  if (motionUpToDate(MOTION_LEFT)) return;
  I2CLock lock(300);   // zapis do PCF pod blokadą I2C; po 300 ms piszemy mimo wszystko (stop ma priorytet)
  //Wheel 1
  pcf8574.digitalWrite(0, LOW);
  pcf8574.digitalWrite(1, HIGH);
  //Wheel 2
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW);
  //Wheel 3
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW);
  //Wheel 4
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH);
}

void turn_right() {
  busy_motors = true;
  busy_forward = false;
  busy_backward = false;
  direction = "W prawo";
  if (motionUpToDate(MOTION_RIGHT)) return;
  I2CLock lock(300);   // zapis do PCF pod blokadą I2C; po 300 ms piszemy mimo wszystko (stop ma priorytet)
  //Wheel 1
  pcf8574.digitalWrite(0, HIGH);
  pcf8574.digitalWrite(1, LOW);
  //Wheel 2
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH);
  //Wheel 3
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH);
  //Wheel 4
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW);
}

void stop_driving() {
  busy_motors = false;
  busy_forward = false;
  busy_backward = false;
  direction = "Stop";
  if (motionUpToDate(MOTION_STOP)) return;
  I2CLock lock(300);   // zapis do PCF pod blokadą I2C; po 300 ms piszemy mimo wszystko (stop ma priorytet)
  //Wheel 1
  pcf8574.digitalWrite(0, LOW);
  pcf8574.digitalWrite(1, LOW);
  //Wheel 2
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, LOW);
  //Wheel 3
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, LOW);
  //Wheel 4
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, LOW);
}
