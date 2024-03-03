/*
  Smart Wireless Car Project Version 2.0
  (Based on the CSS 427 AU23 Wireless Car Project)
  2024 William Humarang

  Project Component: Drive Controller

  References Used:
  1. "WheelDrive Arduino Sketch" by Garth Zeglin
      https://courses.ideate.cmu.edu/16-223/f2016/text/lib/WheelDrive.html#wheeldrive-doxygen
  2. "Complete Guide for Ultrasonic Sensor HC-SR04 with Arduino" by Rui Santos
      https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
  3. 


*/

#include "lib_motor.h"
#include "lib_ultrasonic_sensor.h"

motorControl_t motorLF, motorRF, motorLR, motorRR;

void setup() {
  // Assign all motor pins as outputs
  pinMode(MOTOR_LF_1, OUTPUT);
  pinMode(MOTOR_LF_2, OUTPUT);
  pinMode(MOTOR_RF_1, OUTPUT);
  pinMode(MOTOR_RF_2, OUTPUT);
  pinMode(MOTOR_LR_1, OUTPUT);
  pinMode(MOTOR_LR_2, OUTPUT);
  pinMode(MOTOR_RR_1, OUTPUT);
  pinMode(MOTOR_RR_2, OUTPUT);

  // Set the ultrasonic sensor trigger and echo pins
  pinMode(TRIG_SEN1, OUTPUT);
  pinMode(TRIG_SEN2, OUTPUT);
  pinMode(ECHO_SEN1, INPUT);
  pinMode(ECHO_SEN2, INPUT);

  // Ensure that the motors are idle on startup
  digitalWrite(MOTOR_LF_1, LOW);
  digitalWrite(MOTOR_LF_2, LOW);
  digitalWrite(MOTOR_RF_1, LOW);
  digitalWrite(MOTOR_RF_2, LOW);
  digitalWrite(MOTOR_LR_1, LOW);
  digitalWrite(MOTOR_LR_2, LOW);
  digitalWrite(MOTOR_RR_1, LOW);
  digitalWrite(MOTOR_RR_2, LOW);
  
  Serial.begin(9600);
}

void loop() {
  // Generate a fixed motion sequence to demonstrate the motor modes.

  // Ramp speed up.
  for (int i = 0; i < 11; i++) {
    setFourWheelDriveSpeedAndDelay(25*i, 500);
  }
  // Full speed forward.
  setFourWheelDriveSpeedAndDelay(255, 2000);

  // Ramp speed into full reverse.
  for (int i = 0; i < 21 ; i++) {
    setFourWheelDriveSpeedAndDelay(255 - 25*i, 500);
  }

  // Full speed reverse.
  setFourWheelDriveSpeedAndDelay(-255, 2000);

  // Stop.
  setFourWheelDriveSpeedAndDelay(0, 2000);

  // Full speed, forward, turn, reverse, and turn for a two-wheeled base.
  setFourWheelDriveSpeedAndDelay(255, 2000);
  setFourWheelDriveSpeedAndDelay(0, 1000);
  setFourWheelDriveSpeedAndDelay(-255, 2000);
  setFourWheelDriveSpeedAndDelay(0, 1000);
  setFourWheelDriveSpeedAndDelay(-255, 2000);
  setFourWheelDriveSpeedAndDelay(0, 1000);
  setFourWheelDriveSpeedAndDelay(255, 2000);
  setFourWheelDriveSpeedAndDelay(0, 1000);

  // and repeat
}

/*
  Initialize motor parameters
*/
void motorInit() {
  motorLF.speed = 0;
  motorRF.speed = 0;
  motorLR.speed = 0;
  motorRR.speed = 0;

  // Left Front motor pin configurations
  motorLF.motorOutputs.pinA = MOTOR_LF_1;
  motorLF.motorOutputs.pinB = MOTOR_LF_2;

  // Right Front motor pin configurations
  motorRF.motorOutputs.pinA = MOTOR_RF_1;


}

/*
  Send a PWM value to a motor to drive it at a certain speed
*/
void setMotorPWM(int16_t pwm, motorControl_t motor) {
  if (pwm < 0) {  // reverse speeds
    analogWrite(motor.motorOutputs.pinA, -pwm);
    digitalWrite(motor.motorOutputs.pinB, LOW);
  } else { // stop or forward
    digitalWrite(motor.motorOutputs.pinA, LOW);
    analogWrite(motor.motorOutputs.pinB, pwm);
  }
}

/*
  Set the speed of all 4 motors
*/
void setFourWheelDriveSpeed(uint16_t pwm) {
  setMotorPWM(pwm, motorLF);
  setMotorPWM(pwm, motorRF);
  setMotorPWM(pwm, motorLR);
  setMotorPWM(pwm, motorRR);

  // Print a status message to the console.
  Serial.print("Set motor PWM = ");
  Serial.print(pwm);
}

/*
  Set the speed of all 4 motors for a set amount of time
*/
void setFourWheelDriveSpeedAndDelay(uint16_t pwm, uint32_t duration) {
  setFourWheelDriveSpeed(pwm);
  delay(duration);
}

/*
  Configure an ultrasonic sensor to perform a reading by triggering it with
  a HIGH pulse of 10us
*/
void ultrasonicInit(ultrasonicSensor_t sensor) {
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.trigPin, LOW);
}

/*
  Read an ultrasonic sensor
*/
void ultrasonicRead(ultrasonicSensor_t sensor) {
  // Find the duration between the trigger and echo signals for the pin
  int32_t duration = pulseIn(sensor.echoPin, HIGH);

  // Save the distance in cm
  sensor.distance = (duration / 2) / 29.1;
}