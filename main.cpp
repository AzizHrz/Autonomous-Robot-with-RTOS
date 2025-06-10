#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// **Pin Definitions** don't use 6to11
#define TRIGGER_PIN 5   // Ultrasonic sensor trigger pin
#define ECHO_PIN 18     // Ultrasonic sensor echo pin
#define SPEEDPIN_L 32   // Left motor PWM pin
#define IN3 14          // Left motor direction pin 1
#define IN4 27          // Left motor direction pin 2
#define IN1 25          // Right motor direction pin 1
#define IN2 26          // Right motor direction pin 2
#define SPEEDPIN_R 33    // Right motor PWM pin

#define ENCA_L 2        // Left encoder channel A
#define ENCB_L 16       // Left encoder channel B
#define ENCA_R 4        // Right encoder channel A
#define ENCB_R 17        // Right encoder channel B

// **Shared Variables**


volatile float distance = 0;      // Distance from ultrasonic sensor



bool controlPositionLeft = false;   // Left motor position control flag
bool controlPositionRight = false;  // Right motor position control flag

float targetVelocityLeft = 0;       // Left motor target velocity
float targetVelocityRight = 0;      // Right motor target velocity

// **Synchronization Primitives**
SemaphoreHandle_t mutexDistance;    // Mutex for distance data

SemaphoreHandle_t mutexControl;   // Mutex for control variables

// **Task Handle**
TaskHandle_t mainLoopTaskHandle;  // Handle for main loop task

float pidPositionLeft(int target, int pos);
float pidVelocityLeft(float target, float vel);
float pidPositionRight(int target, int pos);
float pidVelocityRight(float target, float vel);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void ultrasonicTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void mainLoopTask(void *pvParameters);
void debugTask(void *pvParameters);

// **Setup Function**
void setup() {
    Serial.begin(115200);

    // Initialize pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(SPEEDPIN_L, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(SPEEDPIN_R, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENCA_L, INPUT);
    pinMode(ENCB_L, INPUT);
    pinMode(ENCA_R, INPUT);
    pinMode(ENCB_R, INPUT);

    // Create mutexes
    mutexDistance = xSemaphoreCreateMutex();
   
    mutexControl = xSemaphoreCreateMutex();

    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic Task", 32786, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(motorControlTask, "Motor Control Task", 32786, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(mainLoopTask, "Main Loop Task", 32786, NULL, 2, NULL, 1);
    //xTaskCreatePinnedToCore(mainLoopTask, "Main Loop Task", 8192, NULL, 1, &mainLoopTaskHandle, 0);
    // Create debugging task, pinned to core 0
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 32786, NULL, 0, NULL, 1);
}

// **Loop Function**
void loop() {
    // Empty, as all work is handled by FreeRTOS tasks
}

// **Motor Control Function**
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwm, constrain(pwmVal, 0, 80));
}

// **Ultrasonic Task**
void ultrasonicTask(void *pvParameters) {
    Serial.print("Ultrasonic Task ||| core ");
    Serial.println(xPortGetCoreID());
    while (1) {
        Serial.println("Ultrasonic beginnnn");
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
        long duration = pulseIn(ECHO_PIN, HIGH);
        float dist = duration * 0.034 / 2;

        if (xSemaphoreTake(mutexDistance, portMAX_DELAY) == pdTRUE) {
            distance = dist;
            xSemaphoreGive(mutexDistance);
        }

       // xTaskNotifyGive(mainLoopTaskHandle);
        Serial.println("Ultrasonic outttt");
        vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 50ms
    }
}

// **Motor Control Task**
void motorControlTask(void *pvParameters) {
    Serial.print("Motor Control Task ||| core ");
    Serial.println(xPortGetCoreID());
    while (1) {
        Serial.println("Motor beginnnn");
        // Read control variables
        bool ctrlPosL, ctrlPosR;
        int tgtPosL, tgtPosR;
        float tgtVelL, tgtVelR;
        if (xSemaphoreTake(mutexControl, portMAX_DELAY) == pdTRUE) {
            tgtVelL = targetVelocityLeft;
            tgtVelR = targetVelocityRight;
            xSemaphoreGive(mutexControl);
        }

        // Apply to left motor
        int dirLeft = (tgtVelL >= 0) ? 1 : -1;
        int pwrLeft = abs(tgtVelL);
        setMotor(dirLeft, pwrLeft, SPEEDPIN_L, IN3, IN4);

        // Apply to right motor
        int dirRight = (tgtVelR >= 0) ? 1 : -1;
        int pwrRight = abs(tgtVelR);
        setMotor(dirRight, pwrRight, SPEEDPIN_R, IN1, IN2);
        Serial.println("Motor outtttt");

        vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 10ms
    }
}

// **Main Loop Task**
void mainLoopTask(void *pvParameters) {
    Serial.print("Main Loop Task ||| core ");
    Serial.println(xPortGetCoreID());
    while (1) {
        // Wait for notification from ultrasonic task
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("notification main beginnnnnnnn");

        // Read distance
        float dist;
        if (xSemaphoreTake(mutexDistance, portMAX_DELAY) == pdTRUE) {
            dist = distance;
            xSemaphoreGive(mutexDistance);
        }
        dist= dist>50 ? 0 : constrain(dist,0,30); // Default velocity
        
        // Decision logic based on distance
        if (dist < 7 && !controlPositionLeft) {
            if (xSemaphoreTake(mutexControl, portMAX_DELAY) == pdTRUE) {
                controlPositionRight = true;
                controlPositionLeft = true;
                xSemaphoreGive(mutexControl);
            }
        } else if (dist >= 7) {
            if (xSemaphoreTake(mutexControl, portMAX_DELAY) == pdTRUE) {
                controlPositionLeft = false;
                controlPositionRight = false;
                xSemaphoreGive(mutexControl);
            }
        }

        // Set velocity targets
        if (xSemaphoreTake(mutexControl, portMAX_DELAY) == pdTRUE) {
            targetVelocityLeft = controlPositionLeft ? 0 : map(dist,0,30,20,80); // Default velocity
            targetVelocityRight = controlPositionLeft ? 0 : map(dist,0,30,20,80); // Stop right motor if turning right
            xSemaphoreGive(mutexControl);
        }
        Serial.println("Mainnn outttttttt");
        vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 50ms

    }
}

void debugTask(void *pvParameters) {
    while (1) {
        // Access distance with mutex protection
        if (xSemaphoreTake(mutexDistance, portMAX_DELAY) == pdTRUE) {
            Serial.print("Distance: ");
            Serial.println(distance);
            xSemaphoreGive(mutexDistance);
        }

       

        // Access motor variables with mutex protection
        if (xSemaphoreTake(mutexControl, portMAX_DELAY) == pdTRUE) {
            Serial.print("Left Motor - Target vitesse: ");
            Serial.print(targetVelocityLeft);
            

            Serial.print("Right Motor - Target vitesse: ");
            Serial.print(targetVelocityRight);
         

            xSemaphoreGive(mutexControl);
        }

        // Delay for 1 second between prints
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}