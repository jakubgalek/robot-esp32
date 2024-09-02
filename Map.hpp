#include <Servo_ESP32.h>

Servo_ESP32 servoMotor;

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
            read_two_sensors();
            radarData[i][0] = measurement1;
            radarData[i][1] = measurement2;
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
            read_two_sensors();
            radarData[i][0] = measurement1;
            radarData[i][1] = measurement2;
            xSemaphoreGive(xMutex);
        } else {
            Serial.println("Failed to take semaphore");
        }

        servoMotor.write(i);
        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
    }
}






#define SAFE_DISTANCE 30
#define TURN_ANGLE 90
#define DRIVE_DISTANCE 15  // Distance to travel before the next decision in cm
#define BACKUP_DISTANCE 20 // Distance the robot reverses when encountering a difficult space
#define MAX_ATTEMPTS 4     // Maximum number of turn attempts before backing up

void automatic_drive() {

    int turnAttempts = 0; // Turn attempt counter

    read_seven_sensors();
    
    float frontLeft = measurement3;
    float frontCenter = measurement4;
    float frontRight = measurement5;

    // Check for obstacles in front
    if (frontCenter < SAFE_DISTANCE || frontLeft < SAFE_DISTANCE || frontRight < SAFE_DISTANCE) {
        // Obstacle in front
        stop_driving();
        direction = "Stop";
        
        // Decide on a turn
        if (frontLeft > SAFE_DISTANCE && frontRight > SAFE_DISTANCE) {
            // If both left and right sides in front are clear, turn left
            turn(turn_left, TURN_ANGLE, 255);
            direction = "Left";
            turnAttempts++;
        } else if (frontCenter < SAFE_DISTANCE && frontLeft < SAFE_DISTANCE && frontRight < SAFE_DISTANCE) {
            // If the front is blocked, turn towards the side with more space
            if (frontLeft < frontRight) {
                // Turn right if the left side is more blocked
                turn(turn_right, TURN_ANGLE / 2, 255);
                direction = "Right";
            } else {
                // Turn left if the right side is more blocked
                turn(turn_left, TURN_ANGLE / 2, 255);
                direction = "Left";
            }
            turnAttempts++;
        } else if (frontLeft < SAFE_DISTANCE) {
            // If the left side is blocked, turn right
            turn(turn_right, TURN_ANGLE / 2, 255);
            direction = "Right";
            turnAttempts++;
        } else if (frontRight < SAFE_DISTANCE) {
            // If the right side is blocked, turn left
            turn(turn_left, TURN_ANGLE / 2, 255);
            direction = "Left";
            turnAttempts++;
        } else {
            // If none of the above conditions are met, stop
            stop_driving();
            direction = "Stop";
            // delay(1000);  // Wait a moment to assess the situation
        }

        // If the robot has tried turning multiple times and still cannot proceed, reverse
        if (turnAttempts >= MAX_ATTEMPTS) {
            stop_driving();
            direction = "Stop";
            // delay(1000);  // Wait a moment
            drive(backward, BACKUP_DISTANCE, 255); // Reverse
            direction = "Backward";
            // delay(1000);  // Wait a moment to allow the robot to reverse
            turn(turn_right, 180, 255); // Turn 180 degrees
            direction = "Right";
            // delay(1000);  // Wait a moment to allow the robot to turn
            turnAttempts = 0; // Reset the attempt counter
        }
    } else {
        // No obstacles in front
        drive(forward, DRIVE_DISTANCE, 255);
        direction = "Forward";
        turnAttempts = 0; // Reset the attempt counter when the robot can move forward
    }
}

/*

#define SAFE_DISTANCE 30
#define DESIRED_DISTANCE 20 // Odległość, którą robot chce utrzymać od ściany
#define TURN_ANGLE 90
#define DRIVE_DISTANCE 15
#define BACKUP_DISTANCE 20
#define MAX_ATTEMPTS 4

void automatic_drive() {
    int turnAttempts = 0; // Licznik prób skrętu
    float frontLeft = measurement3;
    float frontCenter = measurement4;
    float frontRight = measurement5;
    float sideRight = measurement6; // Czujnik na prawym boku
    float sideLeft = measurement7;  // Czujnik na lewym boku

    // Czy ściana jest z prawej strony?
    if (sideRight < DESIRED_DISTANCE) {
        // Ściana jest zbyt blisko, skręć w lewo
        turn(turn_left, TURN_ANGLE / 2, 255);
        direction = "Left";
    } 
    // Czy ściana jest z lewej strony?
    else if (sideLeft < DESIRED_DISTANCE) {
        // Ściana jest zbyt blisko, skręć w prawo
        turn(turn_right, TURN_ANGLE / 2, 255);
        direction = "Right";
    } 
    // Brak przeszkód z boku
    else {
        // Sprawdź przeszkody z przodu
        if (frontCenter < SAFE_DISTANCE || frontLeft < SAFE_DISTANCE || frontRight < SAFE_DISTANCE) {
            stop_driving();
            direction = "Stop";
            
            // Jeśli przeszkoda jest z przodu, a czujniki boczne pokazują wolne miejsce
            if (frontLeft < SAFE_DISTANCE && frontRight < SAFE_DISTANCE) {
                // Jeśli frontLeft i frontRight są blokowane, ale boczne czujniki pokazują wolne miejsce
                if (sideRight > sideLeft) {
                    // Skręć w lewo, jeśli jest więcej miejsca z lewej strony
                    turn(turn_left, TURN_ANGLE / 2, 255);
                    direction = "Left";
                } else {
                    // Skręć w prawo, jeśli jest więcej miejsca z prawej strony
                    turn(turn_right, TURN_ANGLE / 2, 255);
                    direction = "Right";
                }
                turnAttempts++;
            } else if (frontLeft < SAFE_DISTANCE) {
                // Jeśli lewa strona jest blokowana, skręć w prawo
                turn(turn_right, TURN_ANGLE / 2, 255);
                direction = "Right";
                turnAttempts++;
            } else if (frontRight < SAFE_DISTANCE) {
                // Jeśli prawa strona jest blokowana, skręć w lewo
                turn(turn_left, TURN_ANGLE / 2, 255);
                direction = "Left";
                turnAttempts++;
            } else {
                // Jeśli żadna z powyższych opcji nie jest spełniona, zatrzymaj
                stop_driving();
                direction = "Stop";
            }

            // Jeśli robot próbował wielu skrętów i nadal nie może się ruszyć, cofnij
            if (turnAttempts >= MAX_ATTEMPTS) {
                stop_driving();
                direction = "Stop";
                drive(backward, BACKUP_DISTANCE, 255); // Cofnij
                direction = "Backward";
                turn(turn_right, 180, 255); // Obrót o 180 stopni
                direction = "Right";
                turnAttempts = 0; // Reset liczby prób
            }
        } else {
            // Jeśli nie ma przeszkód z przodu
            drive(forward, DRIVE_DISTANCE, 255);
            direction = "Forward";
            turnAttempts = 0; // Reset liczby prób, gdy robot może jechać do przodu
        }
    }
}

*/