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
    setMotorSpeedAndDelay(25*i, 25*i, 500);
  }
  // Full speed forward.
  setMotorSpeedAndDelay(255,255,2000);

  // Ramp speed into full reverse.
  for (int i = 0; i < 21 ; i++) {
    setMotorSpeedAndDelay(255 - 25*i, 255 - 25*i, 500);
  }

  // Full speed reverse.
  setMotorSpeedAndDelay(-255,-255,2000);

  // Stop.
  setMotorSpeedAndDelay(0,0,2000);

  // Full speed, forward, turn, reverse, and turn for a two-wheeled base.
  setMotorSpeedAndDelay(255, 255, 2000);
  setMotorSpeedAndDelay(0, 0, 1000);
  setMotorSpeedAndDelay(-255, 255, 2000);
  setMotorSpeedAndDelay(0, 0, 1000);
  setMotorSpeedAndDelay(-255, -255, 2000);
  setMotorSpeedAndDelay(0, 0, 1000);
  setMotorSpeedAndDelay(255, -255, 2000);
  setMotorSpeedAndDelay(0, 0, 1000);

  // and repeat
}

/*
  Send a PWM value to a motor to drive it at a certain speed
*/
void setMotorPWM(int pwm, int IN1_PIN, int IN2_PIN) {
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void setMotorSpeed(int pwm_A, int pwm_B) {
  setMotorPWM(pwm_A, MOTOR_LF_1, MOTOR_LF_2);
  setMotorPWM(pwm_A, MOTOR_RF_1, MOTOR_RF_2);
  setMotorPWM(pwm_B, MOTOR_LR_1, MOTOR_LR_2);
  setMotorPWM(pwm_B, MOTOR_RR_1, MOTOR_RR_2);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}

void setMotorSpeedAndDelay(int pwm_A, int pwm_B, int duration) {
  setMotorSpeed(pwm_A, pwm_B);
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