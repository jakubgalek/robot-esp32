#include <ESP32Servo.h>

static const int SERVO_PIN = 25;

int pwmChannelServo = 0;
int pwmFrequencyServo = 50;  // Frequency PWM (standard for servo)

Servo servoMotor;

volatile bool stopFlag = true; // Global variable containing servo state

extern SemaphoreHandle_t xMutex;

void collect_distances_servo() {
    // Move from 0 to 180 degrees
    for (int i = 0; i <= 180; i++) {
        if (stopFlag) {
            servoMotor.write(0);
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                xSemaphoreGive(xMutex);
            }
            //Serial.println("Stopping due to stopFlag");
            return;
        }

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            read_servo_sensors();
            radarData[i][0] = measurement0;
            radarData[i][1] = measurement1;
            xSemaphoreGive(xMutex);
        } else {
            Serial.println("Failed to take semaphore");
        }

        servoMotor.write(i);
        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
    }

    // Move from 180 to 0 degrees
    for (int i = 180; i >= 0; i--) {
        if (stopFlag) {
            servoMotor.write(0);
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                xSemaphoreGive(xMutex);
            }
            //Serial.println("Stopping due to stopFlag");
            return;
        }

        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            read_servo_sensors();
            radarData[i][0] = measurement0;
            radarData[i][1] = measurement1;
            xSemaphoreGive(xMutex);
        } else {
            Serial.println("Failed to take semaphore");
        }

        servoMotor.write(i);
        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
    }
}


#define SAFE_DISTANCE 35
#define TURN_ANGLE 90
#define DRIVE_DISTANCE 15  
#define BACKUP_DISTANCE 20  
#define MAX_ATTEMPTS 4     

void automatic_drive() {
    static int turnAttempts = 0;  // Licznik prób skrętu
    read_five_sensors(); 
    
    float frontLeft = measurement3;
    float frontCenter = measurement4;
    float frontRight = measurement5;

    bool obstacleFront = frontCenter < SAFE_DISTANCE;
    bool obstacleLeft = frontLeft < SAFE_DISTANCE;
    bool obstacleRight = frontRight < SAFE_DISTANCE;

    if (obstacleFront || obstacleLeft || obstacleRight) {
        stop_driving();
        if (!obstacleLeft && obstacleFront && !obstacleRight) {
            // Jeśli przód jest zablokowany, ale boki wolne, skręć losowo
            if (random(0, 2) == 0) {
                turn(turn_left, TURN_ANGLE, 255);
            } else {
                turn(turn_right, TURN_ANGLE, 255);
            }
        } 
        else if (!obstacleLeft) {
            // Skręć w lewo, jeśli lewa strona jest wolna
            turn(turn_left, TURN_ANGLE / 2, 255);
        } 
        else if (!obstacleRight) {
            // Skręć w prawo, jeśli prawa strona jest wolna
            turn(turn_right, TURN_ANGLE / 2, 255);
        } 
        else {
            // Jeśli przód i oba boki są zablokowane
            turnAttempts++;

            if (turnAttempts >= MAX_ATTEMPTS) {
                stop_driving();
                drive(backward, BACKUP_DISTANCE, 255);  
                turn(turn_right, 180, 255);  
                turnAttempts = 0;
            } else {
                // Jeśli jeszcze nie było maksymalnych prób, skręć w miejsce z większą przestrzenią
                if (frontLeft > frontRight) {
                    turn(turn_left, TURN_ANGLE / 2, 255);
                } else {
                    turn(turn_right, TURN_ANGLE / 2, 255);
                }
            }
        }
    } else {
        // Brak przeszkód - jedź do przodu
        forward();
        turnAttempts = 0;  
    }
}
