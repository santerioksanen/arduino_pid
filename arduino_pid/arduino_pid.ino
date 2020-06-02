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

uint32_t next_pid_update = 0;

int16_t set_throttle_value = 15;
uint16_t throttle_val = throttle_still;

uint8_t state=0;    // State 0 = stay still, state 1 = throttle control, state 2 = pid control
uint8_t incomingByte[SERIAL_BUF_SIZE];
boolean received = false;
uint8_t bufIdx = 0;

double rps = 0;
double set_rps = 0;
double steering_angle = 0;
double new_set_rps = 0;

uint8_t state_throttle = STATE_STILL;

SerialParser serial_parser(&steering_angle, &new_set_rps);

uint32_t start_of_loop = 0;
uint32_t next_print = 0;

void loop() {
    start_of_loop = millis();
    run_measurements();
    
    serial_parser.CheckSerial();
    serial_parser.ParseSerial();

    // Calculate rps        
    rps = calculate_rps(state_throttle == STATE_REVERSE);
    steering.SetRelative(steering_angle);

    if(set_rps == 0 && rps == 0){
        state_throttle = STATE_STILL;
    }

    // We have a change in target speed
    if(new_set_rps != set_rps){
        if(new_set_rps == 0){
            set_rps = 0;
            controller.Reset(throttle_still);
            controller.SetMinMaxOutput(THROTTLE_STILL-1, THROTTLE_STILL+1);
        } else if(new_set_rps > 0){
            if(state_throttle == STATE_REVERSE){
                controller.Reset(throttle_still);
                controller.SetMinMaxOutput(THROTTLE_STILL-1, THROTTLE_STILL+1);
                set_rps = 0;
            } else {
                set_rps = new_set_rps;
                controller.SetMinMaxOutput(THROTTLE_STILL, THROTTLE_FULL_POWER);
                state_throttle = STATE_FORWARD;
            }
        } else if(new_set_rps < 0){
            if(state_throttle == STATE_FORWARD){
                controller.Reset(throttle_still);
                controller.SetMinMaxOutput(THROTTLE_STILL-1, THROTTLE_STILL+1);
                set_rps = 0;
            } else {
                set_rps = new_set_rps;
                controller.SetMinMaxOutput(THROTTLE_FULL_REVERSE, THROTTLE_STILL);
                state_throttle = STATE_REVERSE;
            }
        } 
    }

    throttle.SetValue(controller.Compute(set_rps, rps));
    next_pid_update = start_of_loop + PID_UPDATE_INTERVAL; 

    // Send stats over serial
    if(start_of_loop > next_print){
        //Serial.print("Long term average: ");
        //Serial.print(lt_avg);
        //Serial.print(", long term sum: ");
        //Serial.print(lt_sum);
        /*Serial.print("RPS: ");
        Serial.print(rps);
        Serial.print(", Operating mode: ");
        Serial.print(state);
        Serial.print(", Throttle register: ");
        Serial.print(throttle.GetValue());
        Serial.print(", Set RPS value: ");
        Serial.print(set_rps);
        Serial.print(", State throttle: ");
        Serial.print(state_throttle);
        Serial.print(", Set steering angle: ");
        Serial.print(steering_angle);
        Serial.print(", rotation count: ");
        Serial.println(rotation_count);*/
        //Serial.print(", last measured value: ");
        //Serial.println(data_arr[data_point]);
        Serial.print("{\"rps\": ");
        Serial.print(rps);
        Serial.print(", \"set_rps\": ");
        Serial.print(set_rps);
        Serial.print(", \"steering\": ");
        Serial.print(steering_angle);
        Serial.print(", \"revolution_count\": ");
        Serial.print(rotation_count);
        Serial.println("}");
        next_print = start_of_loop+PRINT_INTERVAL;
    }
}
