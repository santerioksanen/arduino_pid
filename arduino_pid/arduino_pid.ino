#include "constants.h"
#include "pid_controller.h"
#include "ldr_speedometer.h"
#include "actuator.h"
#include "serial_parser.h"

// Write default values
uint16_t throttle_still = THROTTLE_STILL;
uint16_t throttle_full_power = THROTTLE_FULL_POWER;
uint16_t throttle_full_reverse = THROTTLE_FULL_REVERSE;

uint16_t steering_right = STEERING_RIGHT;
uint16_t steering_left = STEERING_LEFT;
uint16_t steering_forward = STEERING_FORWARD;

double kp = KP;
double ki = KI;
double kd = KD;

PID controller(PID_UPDATE_INTERVAL, kp, ki, kd, throttle_full_reverse, throttle_full_power, throttle_still);

Servo throttle(THROTTLE_PIN, throttle_full_reverse, throttle_still, throttle_full_power);
Servo steering(STEERING_PIN, steering_right, steering_forward, steering_left);

void setup() {
    cli();
    init_ldr_adc();
    sei();
    Serial.begin(115200);
    init_servo_pwm();
    throttle.Init(throttle_still);
    steering.Init(steering_forward);
    delay(1500);
}

uint32_t lastMillis = 0;
uint32_t next_pid_update = 0;
bool reverse = false;

int16_t set_throttle_value = 15;
uint16_t throttle_val = throttle_still;

uint8_t state=0;    // State 0 = stay still, state 1 = throttle control, state 2 = pid control
uint8_t incomingByte[SERIAL_BUF_SIZE];
boolean received = false;
uint8_t bufIdx = 0;

double rps = 0;
double set_rps = 0;
double steering_angle = 0;

SerialParser serial_parser(&steering_angle, &set_rps);

uint32_t start_of_loop = 0;

void loop() {
    start_of_loop = millis();
    run_measurements();
    
    serial_parser.CheckSerial();
    serial_parser.ParseSerial();

    // Calculate rps        
    rps = calculate_rps(false);
    cli();
    if(reverse == true){
        rps = rps*-1;
    }
    sei();

    steering.SetRelative(steering_angle);
    if(set_rps == 0){
        controller.Reset(throttle_still);
    }
    throttle.SetValue(controller.Compute(set_rps, rps));
    next_pid_update = start_of_loop + PID_UPDATE_INTERVAL; 

    // Send stats over serial
    if((start_of_loop - lastMillis) > PRINT_INTERVAL){
        //Serial.print("Long term average: ");
        //Serial.print(lt_avg);
        //Serial.print(", long term sum: ");
        //Serial.print(lt_sum);
        Serial.print("RPS: ");
        Serial.print(rps);
        Serial.print(", Operating mode: ");
        Serial.print(state);
        Serial.print(", Throttle register: ");
        Serial.print(throttle.GetValue());
        Serial.print(", Set RPS value: ");
        Serial.print(set_rps);
        Serial.print(", Set steering angle: ");
        Serial.print(steering_angle);
        Serial.print(", rotation count: ");
        Serial.println(rotation_count);
        //Serial.print(", last measured value: ");
        //Serial.println(data_arr[data_point]);
        lastMillis = millis();
    }
}
