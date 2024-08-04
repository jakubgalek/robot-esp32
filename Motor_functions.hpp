#pragma once
#include <PCF8574.h>
#include <math.h>

PCF8574 pcf8574(0x20);

#define SPEED_PIN 16
#define ENCODER_PIN 17

const int maxSpeed = 255;  // Maximum speed value (0 - 255)
int pwmChannel = 2;
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
typedef void (*DriveFunction)();
void forward();
void backward();
void turn_left();
void turn_right();
void stop_driving();
 
// Function to drive the robot in different directions for a given distance at a given speed
void drive(DriveFunction directionFunction, double targetDistance, int speed) {
  if (speed < 0 || speed > maxSpeed) {
    Serial.println("Invalid speed value. Speed should be between 0 and 255.");
    return;
  }

  ledcWrite(pwmChannel, speed);
  //Serial.println(speed);
  directionFunction(); // Call the passed function to set the direction

  istargetDistanceTraveled = false; // Reset the flag to indicate target distance is not yet reached

  // Loop until the target distance is reached
  while (!istargetDistanceTraveled) {
    double currentDistancePercentage = totalDistanceTraveled / targetDistance;

    // Check if the specified targetDistance is greater than 90 cm
    if (targetDistance > 90.0) {
      double stopThreshold = targetDistance - 20.0; // Distance to start reducing speed before the end (in centimeters)
      // Reduce the speed if 80% of the stop threshold distance is traveled
      if (totalDistanceTraveled >= stopThreshold) {
        int newSpeed = static_cast<int>(maxSpeed * 0.28); // Reduce speed to 28% of maxSpeed
        ledcWrite(pwmChannel, newSpeed);
         //Serial.print(newSpeed);
        
      }
    } else {
      // Reduce the speed if 80% of the target distance is traveled
      if (currentDistancePercentage >= 0.8) {
        int newSpeed = static_cast<int>(maxSpeed * 0.8); // Reduce speed to 80% of maxSpeed
        ledcWrite(pwmChannel, newSpeed);
        //Serial.print(newSpeed);
      }
    }

    // Check the current state of the indent sensor
    currentIndentState = digitalRead(ENCODER_PIN);
  
    // Detect indent changes
    if (currentIndentState != previousIndentState) {
      if (currentIndentState == HIGH) {
        indents++;
        Serial.print(rotates);
        Serial.print(" rotates ");
        Serial.print(indents);
        Serial.println(" indents");
      }
      previousIndentState = currentIndentState;
    }
  
    // Increment the rotation count when a certain number of indents is reached
    if (indents == 20) {
      rotates++;
      indents = 0;
    }

    // Calculate and update the total distance traveled
    totalDistanceTraveled = rotates * distancePerRotation + (indents * distancePerRotation / 20);

    // Check if the target distance is reached
    if (totalDistanceTraveled >= targetDistance) {
      stop_driving();
      Serial.print("Distance traveled: ");
      Serial.println(totalDistanceTraveled);
      istargetDistanceTraveled = true;
      indents = 0;
      rotates = 0;
      totalDistanceTraveled = 0;
      previousIndentState = false;
      currentIndentState = false;
    }
  }
  if(istargetDistanceTraveled){istargetDistanceTraveled=false; };
  //busy_motors = false;
}

void turn(DriveFunction directionFunction, double angleDegrees, int speed) {
    // Calculate the required distance for the wheels to turn to achieve the desired angle
    double rotationsRequired = angleDegrees / 360.0; // Calculate the fraction of a full rotation
    double distanceRequired = (rotationsRequired * distancePerRotation)*4; // Calculate the distance needed

    // Drive the robot in the specified direction for the calculated distance
    drive(directionFunction, distanceRequired, speed);
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
  //Wheel A
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, HIGH);
  //Wheel B
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH); 
  //Wheel C
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH); 
  //Wheel D
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH); 
}
 
void backward() {
  busy_motors = true;
  direction = "Do tyłu";
  //Wheel A
  pcf8574.digitalWrite(0, HIGH); 
  pcf8574.digitalWrite(1, LOW);
  //Wheel B
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW); 
  //Wheel C
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW); 
  //Wheel D
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW); 
}
 
void turn_left() {
  busy_motors = true;
  direction = "W lewo";
  //Wheel A
  pcf8574.digitalWrite(0, HIGH); 
  pcf8574.digitalWrite(1, LOW);
  //Wheel B
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH); 
  //Wheel C
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH); 
  //Wheel D
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW); 
}
 
void turn_right() {
  busy_motors = true;
  direction = "W prawo";
  //Wheel A
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, HIGH);
  //Wheel B
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW); 
  //Wheel C
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW); 
  //Wheel D
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH); 
}
 
void stop_driving() {
  busy_motors = false;
  busy_forward = false;
  direction = "Stop";
  //Wheel A
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, LOW);
  //Wheel B
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, LOW); 
  //Wheel C
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, LOW); 
  //Wheel D
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, LOW); 
}