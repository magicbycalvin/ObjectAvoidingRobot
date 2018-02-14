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

//#define DEBUG // Uncomment this for serial debugging
#define DRIVE // Uncomment this to drive the motors
 
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define CTRL_PER 25 // Period at which the control system is run [default is 40 Hz, or 25 ms] (ms)
#define DEFAULT_SPEED 150 // Default motor speed [0-255] (DN)
#define SPEED_OF_SOUND 346 // Speed of sound at room temperature (m/s)
#define PULSE_TO 11600 // Timeout for the pulseIn function [11600 times out at a USR distance of 2m] (us)
#define DIST_MAX 400 // Maximum distance the USR can measure, larger values will return a distance of zero (cm)
#define DIST_MIN 2 // Minimum distance the USR can measure, smaller values will return a distance of zero (cm)
#define DIST_THRESH 6 // Any measured distance smaller than this will cause the robot to backup and turn around (cm)
#define ROT_GAIN 10 // Rotational gain to multiply the difference of the measured distances to determine how aggresive the turn should be

// USR trigger and echo pins
#define TRIG1 2
#define ECHO1 3
#define TRIG2 4
#define ECHO2 5
#define TRIG3 6
#define ECHO3 7

// Since we don't have a feedback loop, we may need to tune the motor speeds to each other (offsets are in DN [0-255])
#define M1_OFFSET 13 // Back Left
#define M2_OFFSET 0 // Back Right
#define M3_OFFSET 0 // Front Right
#define M4_OFFSET 13 // Front Left

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create 4 motor objects
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

// Enumerate the three USRs
enum USR {
  USR1,
  USR2,
  USR3
};

// Variables
#ifdef DEBUG
  uint8_t debugCount = 0;
#endif

float distL, distR, distF;
unsigned long curTime;
unsigned long lastTime = 0;
//uint8_t speedL, speedR;

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("DEBUG ON");
    Serial.println("Dist L\tDist F\tDist R\tMotor L\tMotor R");
    Serial.println("---");
  #endif

  // Set the apropriate input and output pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

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

  // Delay for a second so that we can set the robot down if it was being programmed
  delay(1000);
  
}

void loop() {

  curTime = millis();

  // Run the control system at a known frequency
  if ( curTime - lastTime >= CTRL_PER ) {

    // Read USR distances
    distL = readUSR( USR1 );
    distF = readUSR( USR2 );
    distR = readUSR( USR3 );

    // Motor driving algorithm
    if ( distL >= DIST_MAX && distF >= DIST_MAX && distR >= DIST_MAX ) {
      
      driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );
      
    } else if ( distF < DIST_THRESH ){
      
      backUp();
      rotateL();
      
    } else if ( distL < DIST_THRESH ) {
      
       backUp();
       rotateR();
       
    } else if ( distR < DIST_THRESH ) {
      
      backUp();
      rotateL();
      
    } else {
      
      driveMotors( DEFAULT_SPEED / 2, FORWARD, DEFAULT_SPEED / 2, FORWARD );
      
    }
    
    /*else {
      
      if ( distR < distL ) {
        driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED + ROT_GAIN*(distL-distR), FORWARD );
      } else if ( distR > distL ) {
        driveMotors( DEFAULT_SPEED + ROT_GAIN*(distR-distL), FORWARD, DEFAULT_SPEED, FORWARD );
      } else {
        driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );
      }
      
    }*/

    // Update the time
    lastTime = curTime;
  
    #ifdef DEBUG
      if ( !(debugCount%100) ) {
        Serial.println("---");
        Serial.println("Dist L\tDist F\tDist R\tMotor L\tMotor R");
        Serial.println("---");
      }
      Serial.print(distL);
      Serial.print('\t');
      Serial.print(distF);
      Serial.print('\t');
      Serial.println(distR);
      debugCount++;
    #endif
    
  }
  
}

/*
 * readUSR
 * 
 * Reads data from the USR sensors. According to the datasheet, the USRs should not be read
 * higher than a frequency of 40 Hz. Because of that, this function should not be called faster
 * than that (1/40 Hz = 25 ms).
 * 
 * INPUTS:
 *  num - USR index number
 *  
 * OUTPUTS:
 *  distance - Distance, in cm, to the object in front of the USR. According to the datasheet,
 *    it is constrained from 2 cm to 400 cm with a resolution of 0.3 cm.
 */
float readUSR( uint8_t num ) {

  float distance;
  unsigned long duration;
  int trigPin, echoPin;

  // Decide which USR we are reading
  switch (num) {
    case USR1:
      trigPin = TRIG1;
      echoPin = ECHO1;
      break;
    case USR2:
      trigPin = TRIG2;
      echoPin = ECHO2;
      break;
    case USR3:
      trigPin = TRIG3;
      echoPin = ECHO3;
      break;
    default:
      return 0;
      break;
  }

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin by returning the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, PULSE_TO);
  
  // Calculate the distance in cm (divide by 2 for sound to reach object and come back)
  // 1us * (1s/1000000us) * 346m/s * 100cm/1m / 2 = 1us * 346m/s / 20,000 
  distance = duration * SPEED_OF_SOUND / 20000.0;

  if ( distance > DIST_MAX ) distance = DIST_MAX;
  else if ( distance < DIST_MIN ) distance = 0;
  
  return (distance) ? distance : DIST_MAX;
  
}

/*
 * driveMotors
 * 
 * Drives the motors using a tank setup. Assumes motors 1 and 4 are on the left side and motors
 * 2 and 3 are on the right side. DRIVE must be defined in order to drive the motors. If it is not,
 * the motors will not turn.
 * 
 * INPUTS:
 *  speedL - Left side speed in DN. Can range from 0 to 255.
 *  dirL - Direction of the left wheels where 0 is backward and 1 is forward
 *  speedR - Same as speedL but for right side
 *  dirR - Same as dirL but for right side
 */
void driveMotors( uint8_t speedL, uint8_t dirL, uint8_t speedR, uint8_t dirR ) {

  #ifdef DRIVE
  
    // For the robot, motors 1 and 4 are on the left, motors 2 and 3 are on the right
    // Set left speed
    motor1->setSpeed(speedL+M1_OFFSET);
    motor4->setSpeed(speedL+M4_OFFSET);
    // Set right speed
    motor2->setSpeed(speedR+M2_OFFSET);
    motor3->setSpeed(speedR+M3_OFFSET);
  
    // Set direction
    // Left motors
    motor1->run(dirL);
    motor4->run(dirL);
    // Right motors
    motor2->run(dirR);
    motor3->run(dirR);
    
  #else
  
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motor4->run(RELEASE);
    
  #endif

  return;
  
}

void backUp() {
  driveMotors( DEFAULT_SPEED, BACKWARD, DEFAULT_SPEED, BACKWARD );
  delay(1250);
  return;
}

void rotateR() {
  driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, BACKWARD );
  delay(750);
  return;
}

void rotateL() {
  driveMotors( DEFAULT_SPEED, BACKWARD, DEFAULT_SPEED, FORWARD );
  delay(750);
  return;
}












