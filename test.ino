#include <Servo.h>
#include <SoftwareSerial.h>
#include "pins.h"

// ===================== Pin Assignments and Constants =====================
#define ENABLE_LEFT PWM1
#define ENABLE_RIGHT PWM2
#define TRIG_PIN D7
#define ECHO_PIN D8
#define BLUETOOTH_RX D10
#define BLUETOOTH_TX D11

int motorSpeed = 150;
int distanceThreshold = 20; 
char command;

// ===================== Setup Functions =====================
void setupMotorPins() {
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
    pinMode(ENABLE_LEFT, OUTPUT);
    pinMode(ENABLE_RIGHT, OUTPUT);
    analogWrite(ENABLE_LEFT, motorSpeed);
    analogWrite(ENABLE_RIGHT, motorSpeed);
}

void setupSensorPins() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void setupBluetooth() {
    SoftwareSerial bluetooth(BLUETOOTH_RX, BLUETOOTH_TX);
    bluetooth.begin(9600);
}

void setup() {
    Serial.begin(9600);  // For debugging
    setupMotorPins();
    setupSensorPins();
    setupBluetooth();
}

// ===================== Motor Control Functions =====================
void moveForward() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void moveBackward() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void turnRight() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void stopMotors() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// ===================== Sensor Functions =====================
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2;
    return distance;
}

void handleCollisionAvoidance(int distance) {
    if (distance < distanceThreshold) {
        stopMotors();
        moveBackward();
        delay(500);
        turnRight();
        delay(500);
    } else {
        moveForward();
    }
}

// ===================== Bluetooth Functions =====================
void handleBluetoothCommand(char cmd) {
    switch (cmd) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnRight(); break;
        case 'R': turnRight(); break;
        case 'S': stopMotors(); break;
        default: stopMotors(); break;
    }
}

void listenToBluetooth() {
    if (Serial.available() > 0) {
        command = Serial.read();
        handleBluetoothCommand(command);
    }
}

// ===================== Main Loop =====================
void loop() {
    int distance = getDistance();
    handleCollisionAvoidance(distance);
    listenToBluetooth();
}
