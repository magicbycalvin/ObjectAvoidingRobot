/*
 * ObjectAvoidingRobot.ino
 * 
 * DATE: 2-11-2018
 * AUTHOR: Calvin Kielas-Jensen
 * CONTRIBUTORS: Hussein, Hussein, Merrick
 * COURSE: ME 492 - Robotic Systems
 * 
 * DESCRIPTION: Uses 3 ultrasonic range sensors to drive a robot around and avoid obstacles. The motor driver board
 *  being used is the Adafruit Motor Driver V2. The ultrasonic range sensors are the HC-SR04 sensors. Four motors
 *  are run simultaneously in tank mode to drive (using differential steering).
 *  
 */

#define DEBUG
 
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define DEFAULT_SPEED 150

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create 4 motor objects
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("DEBUG ON");
  #endif

  // Create with the default frequency 1.6KHz
  AFMS.begin();
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(DEFAULT_SPEED);
  motor2->setSpeed(DEFAULT_SPEED);
  motor3->setSpeed(DEFAULT_SPEED);
  motor4->setSpeed(DEFAULT_SPEED);

  // Turn all four motors off
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  
}

void loop() {
  
}

readUSR( uint8_t num ) {
  
}
















