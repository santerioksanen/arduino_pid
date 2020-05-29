#include "constants.h"
#include "pid_controller.h"
#include "ldr_speedometer.h"
#include "actuator.h"

PID controller(PID_UPDATE_INTERVAL, 1.5, 0.5, 0.1, THROTTLE_FULL_REVERSE, THROTTLE_FULL_POWER, THROTTLE_STILL);

Servo throttle(THROTTLE_PIN);
Servo steering(STEERING_PIN);

void setup() {
    cli();
    init_ldr_adc();
    sei();
    Serial.begin(115200);
    init_servo_pwm();
    throttle.Init(THROTTLE_STILL);
    steering.Init(STEERING_FORWARD);
    delay(1500);
    Serial.print("OCRI1B: ");
    Serial.print(OCR1B);
    Serial.print(", OCR1A: ");
    Serial.println(OCR1A);
}

uint32_t lastMillis = 0;
uint32_t next_pid_update = 0;

uint16_t set_throttle_value = 15;
uint16_t throttle_val = THROTTLE_STILL;

double rps = 0;

uint8_t state=0;    // State 0 = stay still, state 1 = throttle control, state 2 = pid control
uint8_t incomingByte[SERIAL_BUF_SIZE];
boolean received = false;
uint8_t bufIdx = 0;

void process_incoming_data(){
    if(received){
        switch(incomingByte[0]){
        case 'I':
            state = 1;
            throttle_val = OCR1A+1;
            break;
        //case 'R':
        //    state = 1;
        //    throttle_val = OCR1A-1;
        //    break;
        case 'S':
            state = 0;
            throttle_val = THROTTLE_STILL;
            break;
        case 'P':
            state = 2;
            set_throttle_value = 30;
            //steering.SetValue(STEERING_RIGHT);
            break;
        case 'L':
            steering.SetValue(STEERING_LEFT);
            break;
        case 'R':
            steering.SetValue(STEERING_RIGHT);
            break;
        case 'F':
            steering.SetValue(STEERING_FORWARD);
            break;
            /*switch(incomingByte[1]){
            case 'T':   // Set Throttle (ST)
                state = 1;
                throttle = 
                break;
            }
            break;*/
        }
        received = false;
        bufIdx = 0;
    }
}

uint32_t start_of_loop = 0;

void loop() {
    start_of_loop = millis();

    run_measurements();

    // Parse serial data
    while (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte[bufIdx++] = Serial.read();
        if(incomingByte[bufIdx-1] == '\n'){
        received = true;
        }
    }

    process_incoming_data();
    
    // Calculate rps
    if(start_of_loop >= next_pid_update){
        bool print_details = false;
        if(state == 2) print_details=true;
        
        rps = calculate_rps(print_details);
    }
    
    // Update throttle for pid
    switch(state){
        case 0:
        if(OCR1A != THROTTLE_STILL){
            OCR1A = THROTTLE_STILL;
        }
        break;
        case 1:
        if(OCR1A != throttle_val){
            OCR1A = throttle_val;
        }
        break;
        case 2:
        if(start_of_loop >= next_pid_update){
            throttle.SetValue(controller.Compute(set_throttle_value, rps));
            next_pid_update = millis() + PID_UPDATE_INTERVAL;
        }
        break;
    }

    

    // Send stats over serial
    if((millis() - lastMillis) > PRINT_INTERVAL){
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
        Serial.print(", rotation count: ");
        Serial.println(rotation_count);
        //Serial.print(", last measured value: ");
        //Serial.println(data_arr[data_point]);
        lastMillis = millis();
    }
}
