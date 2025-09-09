#pragma once
#include <PCF8574.h>
#include <math.h>
#include <Wire.h>
#include <MPU6050_light.h>

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
String direction = "Stop";
  
// Dane robota dla jazdy liniowej
double robotX = 0.0;
double robotY = 0.0;
double robotAngle = 0.0; // kąt w stopniach

// Parametry fizyczne
const double distancePerRotation = 22.3;   // Obwód koła [cm]
const int encoderIndentsPerRotation = 20;    // Liczba impulsów na obrót koła
// Każdy impuls odpowiada przesunięciu (w cm):
const double distancePerIndent = (distancePerRotation / encoderIndentsPerRotation)/2; // 22.3/20 ≈ 1.115 cm

// Parametr do obrotu – efektywny promień skrętu robota [cm]. 
// (Dla robota obracającego się w miejscu będzie to odległość od środka obrotu do punktu styku koła)
const double turningRadius = 7.5;

// Deklaracje funkcji sterujących
typedef void (*DirectionFunction)();
void forward();
void backward();
void turn_left();
void turn_right();
void stop_driving();

typedef void (*TurnFunction)();

// Funkcja przerwania enkodera z debouncingiem
void encoderInterrupt() {
  unsigned long interruptTime = micros();
  if (interruptTime - lastInterruptTime > debounceDelayMicros) {
    indents++;  // Zliczamy impuls, jeśli minął czas debouncigu
    lastInterruptTime = interruptTime;
  }
}

// Funkcja aktualizująca pozycję robota dla jazdy do przodu/tyłu
void updatePosition(double startX, double startY, double distance, DirectionFunction dir) {
  // Przyjmujemy, że dla jazdy do przodu kierunek dodatni, a do tyłu ujemny
  double directionCoeff = (dir == forward) ? 1 : -1;
  robotX = startX + distance * cos(radians(robotAngle)) * directionCoeff;
  robotY = startY + distance * sin(radians(robotAngle)) * directionCoeff;
}

// Funkcja jazdy liniowej wykorzystująca przelicznik impulsów (1 impuls = 1,115 cm)
void drive(DirectionFunction directionFunc, double targetDistance, int speed) {
  if (speed < 0 || speed > maxSpeed) return;

  // Ustawiamy prędkość i kierunek jazdy
  ledcWrite(pwmChannelSpeed, speed);
  directionFunc();  // np. forward() lub backward()

  double startX = robotX;
  double startY = robotY;
  double distanceTraveled = 0.0;
  
  // Reset licznika impulsów
  noInterrupts();
  indents = 0;
  interrupts();

  // Pętla zliczająca impulsami przejechany dystans
  while (fabs(distanceTraveled) < fabs(targetDistance)) {
    // Każdy impuls przekłada się na distancePerIndent [cm]
    distanceTraveled = indents * distancePerIndent;
    updatePosition(startX, startY, distanceTraveled, directionFunc);
  }

  stop_driving();
  // Korekta końcowej pozycji
  updatePosition(startX, startY, targetDistance, directionFunc);
}

// Funkcja obrotu – obliczamy kąt na podstawie przebytego dystansu
MPU6050 mpu(Wire);
unsigned long lastGyroTime = 0;
float currentAngle = 0;

void turn(TurnFunction turnDirection, double targetAngleDegrees, int speed) {
  currentAngle = 0;

  // Używamy micros() dla precyzyjnego pomiaru czasu
  unsigned long lastGyroTime = micros();

  // Start silników i kierunku obrotu
  ledcWrite(pwmChannelSpeed, speed);
  turnDirection();

  while (fabs(currentAngle) < fabs(targetAngleDegrees)) {
    mpu.update();

    float gyroZ = mpu.getGyroZ(); // prędkość obrotowa [°/s]

    unsigned long nowMicros = micros();
    float deltaTime = (nowMicros - lastGyroTime) / 1000000.0; // w sekundach
    lastGyroTime = nowMicros;

    // Ograniczamy maksymalny deltaTime, żeby uniknąć dużych skoków (np. jeśli coś chwilowo wstrzymało pętlę)
    if (deltaTime > 0.02) deltaTime = 0.02; // max 20 ms

    currentAngle += gyroZ * deltaTime;

    yield();
    //Serial.print("Kąt obrotu: ");
    //Serial.println(currentAngle);
  }

  stop_driving();

  // Aktualizacja kąta globalnego robota
  if (turnDirection == turn_left) {
    robotAngle -= fabs(currentAngle);
  } else {
    robotAngle += fabs(currentAngle);
  }
  robotAngle = fmod(robotAngle, 360.0);
  if (robotAngle < 0) robotAngle += 360.0;
}

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

void forward() {
  busy_motors = true;
  busy_forward = true;
  direction = "Do przodu";
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
  busy_backward = true; 
  direction = "Do tyłu";
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
  direction = "W lewo";
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
  direction = "W prawo";
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