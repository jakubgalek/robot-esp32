#pragma once
#include <PCF8574.h>
#include <math.h>

PCF8574 pcf8574(0x20);

#define SPEED_PIN 16

const int pwmFrequencySpeed = 1000; // (1 kHz)
const int pwmResolution = 8; // (8 bit)

#define ENCODER_PIN 17

const int maxSpeed = 255;  // Maximum speed value (0 - 255)
int pwmChannelSpeed = 9;
int rotates = 0;
int indents = 0;
bool previousIndentState;
bool currentIndentState = false;
double totalDistanceTraveled = 0; // Total distance traveled in centimeters
double istargetDistanceTraveled = false;
const double distancePerRotation = 20.8; // Measured circumference of the wheel in centimeters

bool busy_motors = false; 
bool busy_forward = false; 

String direction="Stop";

// Function type for driving commands
typedef void (*DirectionFunction)(); // Typ dla funkcji kierunku
void forward();
void backward();
void turn_left();
void turn_right();
void stop_driving();
 
 
// Aktualne położenie robota
double robotX = 0.0;
double robotY = 0.0;
double robotAngle = 0.0; // Kąt w stopniach


void updatePosition(double startX, double startY, double distance, DirectionFunction dir) {
  double directionCoeff = (dir == forward) ? 1 : -1;
  robotX = startX + distance * cos(radians(robotAngle)) * directionCoeff;
  robotY = startY + distance * sin(radians(robotAngle)) * directionCoeff;
}

void drive(DirectionFunction directionFunc, double targetDistance, int speed) {
  if (speed < 0 || speed > maxSpeed) return;

  // Ustaw prędkość i kierunek
  ledcWrite(pwmChannelSpeed, speed);
  directionFunc(); // Wywołaj przekazaną funkcję (forward/backward)

  double startX = robotX;
  double startY = robotY;
  double distanceTraveled = 0.0;
  indents = 0;

  while (abs(distanceTraveled) < abs(targetDistance)) {
    currentIndentState = digitalRead(ENCODER_PIN);
    if (currentIndentState != previousIndentState && currentIndentState == HIGH) {
      indents++;
      distanceTraveled = (indents / 20.0) * distancePerRotation;
      
      // Aktualizuj pozycję na bieżąco
      updatePosition(startX, startY, distanceTraveled, directionFunc);
    }
    previousIndentState = currentIndentState;
  }

  stop_driving();
  // Dokładna aktualizacja końcowej pozycji
  updatePosition(startX, startY, targetDistance, directionFunc);
}


typedef void (*TurnFunction)(); // Typ dla funkcji skrętu

void turn(TurnFunction turnDirection, double targetAngleDegrees, int speed) {
  if (speed < 0 || speed > maxSpeed) return;

  // Ustaw prędkość
  ledcWrite(pwmChannelSpeed, speed);

  // Wywołaj odpowiednią funkcję skrętu
  turnDirection();

  double startAngle = robotAngle;
  double angleTraveled = 0.0;
  indents = 0; // Reset enkodera

  while (abs(angleTraveled) < abs(targetAngleDegrees)) {
    // Aktualizacja enkodera (1 indent = 1.8 stopnia dla enkodera 200 CPR)
    currentIndentState = digitalRead(ENCODER_PIN);
    if (currentIndentState != previousIndentState && currentIndentState == HIGH) {
      indents++;
      angleTraveled = indents * 1.8; // Przelicz indenty na stopnie
    }
    previousIndentState = currentIndentState;
  }

  stop_driving();
  // Aktualizuj kąt (uwzględniając kierunek)
  if (turnDirection == turn_left) {
    robotAngle = fmod(startAngle - abs(targetAngleDegrees), 360.0);
  } else {
    robotAngle = fmod(startAngle + abs(targetAngleDegrees), 360.0);
  }
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