/*
  Smart Wireless Car Project Version 2.0
  (Based on the CSS 427 AU23 Wireless Car Project)
  2024 William Humarang

  Project Component: Library header file for defining the HC-SR04
  Ultrasonic Sensor.

  References Used:
  1.

*/

// Ultrasonic sensor pinouts (SEN1 = FRONT, SEN2 = REAR)
#define TRIG_SEN1 14
#define TRIG_SEN2 12
#define ECHO_SEN1 34
#define ECHO_SEN2 35

// Ultrasonic sensor typedef to store sensor attributes
typedef struct ultrasonicSensor_t {
  uint16_t sensor;
  uint8_t trigPin;
  uint8_t echoPin;
  int32_t distance;
} ultrasonicSensor_t;