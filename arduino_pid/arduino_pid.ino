#include "constants.h"
#include "pid_controller.h"
#include "ldr_speedometer.h"

PID controller(PID_UPDATE_INTERVAL, 1.5, 0.5, 0.1, THROTTLE_FULL_REVERSE, THROTTLE_FULL_POWER, THROTTLE_STILL);

void init_pwm() {
  // Set PB1 to be an output (Pin9 Arduino UNO)
  DDRB |= (1 << PB1);

  // Clear Timer/Counter Control Registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Set inverting mode
  TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
  
  TCCR1B = (1 << WGM13) | (1 << CS11);     // Set mode to phase and frequency correct, set prescaler to 8 (16 MHz/8) = 2000 kHz 
  ICR1 = 20000;                            // Count to 20000 (20000 * 1/2000kHz) = 0.01 s = 10 ms
  OCR1A = THROTTLE_STILL;
}

void setup() {
  cli();
  init_ldr_adc();
  init_pwm();
  sei();
  Serial.begin(115200);
  delay(1500);
}

uint32_t lastMillis = 0;
uint32_t next_millis = 0;
uint32_t next_turn_millis = 0;
uint32_t next_pid_update = 0;

uint16_t set_throttle_value = 15;
uint16_t throttle = THROTTLE_STILL;
float error = 0;
float diff_error = 0;

float k_p = 0.3;
float k_d = 0;

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
        throttle = OCR1A+1;
        break;
      case 'R':
        state = 1;
        throttle = OCR1A-1;
        break;
      case 'S':
        state = 0;
        throttle = THROTTLE_STILL;
        break;
      case 'P':
        state = 2;
        set_throttle_value = 30;
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
      if(OCR1A != throttle){
        OCR1A = throttle;
      }
      break;
    case 2:
      if(start_of_loop >= next_pid_update){
        OCR1A = controller.Compute(set_throttle_value, rps);
        next_pid_update = millis() + PID_UPDATE_INTERVAL;
      }
      break;
  }
  
  // Start measurement
  if(millis() >= next_millis){
    ADCSRA |= (1 << ADSC);
    next_millis = next_millis + MEAS_INTERVAL;
  }

  // Clear measurements
  if(millis() - last_rotation > CLEAR_MEASUREMENTS_DELAY){
    cli();
    for(uint8_t i = 0; i < WAVES_NUM; i++){
      waves_arr[i] = 0;
    }
    last_below_avg = 0;
    sei();
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
    Serial.print(", OCR1A: ");
    Serial.print(OCR1A);
    Serial.print(", rotation count: ");
    Serial.println(rotation_count);
    //Serial.print(", last measured value: ");
    //Serial.println(data_arr[data_point]);
    lastMillis = millis();
  }
}
