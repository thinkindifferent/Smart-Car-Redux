/*
  Smart Wireless Car Project Version 2.0
  (Based on the CSS 427 AU23 Wireless Car Project)
  2024 William Humarang

  Project Component: Library header file for defining the TT DC motor.

  References Used:
  1. "WheelDrive Arduino Sketch" by Garth Zeglin
      https://courses.ideate.cmu.edu/16-223/f2016/text/lib/WheelDrive.html#wheeldrive-doxygen
*/

// Motor pinouts
#define MOTOR_LF_1 27
#define MOTOR_LF_2 26
#define MOTOR_RF_1 25
#define MOTOR_RF_2 33
#define MOTOR_LR_1 18
#define MOTOR_LR_2 19
#define MOTOR_RR_1 16
#define MOTOR_RR_2 17

// Enum for drive direction
typedef enum direction_t {
  FORWARD_DRIVE,
  REVERSE_DRIVE,
  LEFT_DRIVE,
  RIGHT_DRIVE
} direction_t;

// Enum for the 4 motors
typedef enum motor_t {
  MOTOR_LF,
  MOTOR_RF,
  MOTOR_LR,
  MOTOR_RR
} motor_t;

// Motor typedef to store attributes
typedef struct motorControl_t {
  motor_t motor;
  int16_t speed; // Speed in PWM
  direction_t direction;
} motorControl_t;