#include <Servo.h>  // For controlling servo motors, if used
#include <SoftwareSerial.h>  // For serial communication with Bluetooth or other peripherals
#include "pins.h"  // Include the pin definition sheet

// Define pin assignments for motors, sensors, and communication using the pin mappings from pins.h
#define MOTOR_LEFT_FORWARD IN1  // Define for H-Bridge, input 1 (D4)
#define MOTOR_LEFT_BACKWARD IN2  // Define for H-Bridge, input 2 (D5)
#define MOTOR_RIGHT_FORWARD IN3  // Define for H-Bridge, input 3 (D6)
#define MOTOR_RIGHT_BACKWARD IN4  // Define for H-Bridge, input 4 (D7)
#define ENABLE_LEFT PWM1  // Define for PWM pin to enable motor speed (D9)
#define ENABLE_RIGHT PWM2  // Define for PWM pin to enable motor speed (D10)
#define TRIG_PIN D7  // Pin 7, used for the ultrasonic sensor trigger
#define ECHO_PIN D8  // Pin 8, used for the ultrasonic sensor echo
#define BLUETOOTH_RX D10  // Pin 10, for Bluetooth RX
#define BLUETOOTH_TX D11  // Pin 11, for Bluetooth TX

// Placeholder for motor speed control (use PWM to adjust speed)
int motorSpeed = 150;  // Speed value from 0 to 255 for PWM

// Placeholder for ultrasonic distance threshold
int distanceThreshold = 20;  // Distance in cm to trigger avoidance behavior

// Placeholder for remote control commands (if using Bluetooth)
char command;  // Variable to store Bluetooth command

// Setup function runs once when the program starts
void setup() {
  // Initialize motor control pins as outputs
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  
  // Initialize PWM enable pins for motor speed control
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Begin serial communication (for Bluetooth or debugging)
  Serial.begin(9600);

  // Set up Bluetooth communication, if applicable
  SoftwareSerial bluetooth(BLUETOOTH_RX, BLUETOOTH_TX);
  bluetooth.begin(9600);

  // Enable motors initially (can be adjusted based on speed)
  analogWrite(ENABLE_LEFT, motorSpeed);
  analogWrite(ENABLE_RIGHT, motorSpeed);
}

// Main loop function runs repeatedly
void loop() {
  // Check distance from ultrasonic sensor
  int distance = getDistance();

  // Collision avoidance logic
  if (distance < distanceThreshold) {
    stopMotors();  // Stop motors if an obstacle is too close
    backward();  // Move backward
    delay(500);  // Wait for half a second
    turnRight();  // Turn right to avoid obstacle
    delay(500);  // Adjust delay time based on turn needs
  } else {
    forward();  // Move forward if no obstacle
  }

  // Placeholder for Bluetooth control
  if (Serial.available() > 0) {
    command = Serial.read();
    controlRobotWithBluetooth(command);  // Function to handle Bluetooth commands
  }
}

// Function to control motors for forward motion (both wheels)
void forward() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Function to control motors for backward motion
void backward() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

// Function to turn right (adjust motor speeds for differential steering)
void turnRight() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// Function to get the distance from ultrasonic sensor
int getDistance() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert the duration into distance in centimeters
  int distance = duration * 0.034 / 2;
  return distance;
}

// Function to control the robot with Bluetooth commands
void controlRobotWithBluetooth(char command) {
  switch (command) {
    case 'F':  // Forward
      forward();
      break;
    case 'B':  // Backward
      backward();
      break;
    case 'L':  // Left turn
      turnRight();  // Adjust if you want to turn left instead
      break;
    case 'R':  // Right turn
      turnRight();
      break;
    case 'S':  // Stop
      stopMotors();
      break;
    default:
      stopMotors();  // Stop if an unrecognized command is received
      break;
  }
}
