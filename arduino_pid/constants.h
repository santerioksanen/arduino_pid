// Includes all constants and variables shared between multiple files
// PINS: PWM out: 9, LDR in: A2P

#ifndef constants_h
#define constants_h

#define ADC_pin B010
#define MEAS_BUF_SIZE 512
#define MEAS_INTERVAL 1
#define AVG_DIFF 7
#define WAVES_NUM 5

#define THROTTLE_PIN 9
#define THROTTLE_STILL 1420
#define THROTTLE_FULL_POWER 2000
#define THROTTLE_FULL_REVERSE 1000

#define STEERING_PIN 10
#define STEERING_RIGHT 1100
#define STEERING_FORWARD 1500
#define STEERING_LEFT 1900

#define PID_UPDATE_INTERVAL 100
#define MAX_STEP_CHANGE 1
#define CLEAR_MEASUREMENTS_DELAY 200
#define PRINT_INTERVAL 1000
#define SERIAL_BUF_SIZE 15


#endif