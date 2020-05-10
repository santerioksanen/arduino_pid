#define ADC_pin B010
#define MEAS_BUF_SIZE 512
#define MEAS_INTERVAL 1
#define AVG_DIFF 5
#define WAVES_NUM 10
#define THROTTLE_PIN 3
#define THROTTLE_STILL 1420
#define THROTTLE_FULL_POWER 2000
#define THROTTLE_FULL_REVERSE 1000
#define PID_UPDATE_INTERVAL 200
#define MAX_STEP_CHANGE 20
#define CLEAR_MEASUREMENTS_DELAY 800
#define PRINT_INTERVAL 1000
#define SERIAL_BUF_SIZE 15

// PINS: PWM out: 9, LDR in: A2P

volatile uint8_t data_arr[MEAS_BUF_SIZE]={0};
volatile uint16_t data_point = 0;
volatile uint32_t lt_sum = 0;
volatile uint32_t meas_count = 0;
volatile uint32_t last_below_avg = 0;
volatile uint32_t last_above_avg = 0;
volatile uint32_t last_rotation = 0;
volatile boolean above_avg = false;
volatile boolean data_flag=false;
volatile uint16_t waves_arr[WAVES_NUM]={0};
volatile uint8_t waves_point = 0;
volatile uint8_t lt_avg = 0;
volatile boolean rotation = false;
volatile uint32_t rotation_count = 0;

ISR(ADC_vect){ //This is our interrupt service routine
  uint8_t tmp = ADCH;
  lt_sum = lt_sum - data_arr[data_point];
  data_arr[data_point] = tmp; // Store the 8 most significant bits in the data buffer and increment the bufferIndex
  lt_sum = lt_sum + tmp;
  meas_count++;
  data_flag = true;
  data_point = (data_point+1) % MEAS_BUF_SIZE;
  lt_avg = lt_sum / MEAS_BUF_SIZE;
  if(tmp < (lt_avg - AVG_DIFF) && above_avg == true){
    if(last_below_avg > 0){
      waves_arr[waves_point] = millis() - last_below_avg;
      waves_point = (waves_point + 1) % WAVES_NUM;
    }
    last_below_avg = millis();
    above_avg = false;
    rotation = true;
    last_rotation = millis();
    rotation_count++;
  } else if(tmp > (lt_avg + AVG_DIFF) && above_avg == false){
    last_above_avg = millis();
    above_avg = true;
  }
}

void init_adc() {
  //Enable ADC and interrupt on conversion done
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  DIDR0  = bit(ADC0D)|bit(ADC1D)|bit(ADC2D)|bit(ADC3D);  // disable digital input buffer for analog input pin 0, 1 and 2

  // Enable ADC, set auto trigger, enable interrupt, set clock divider to 128 = 125 kHz
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //  | (1 << ADATE)
  ADMUX = (1 << REFS0) | (1 << ADLAR); //Set Voltage reference to Avcc (5v), Left adjust converted value

  ADMUX |= ADC_pin;
  
  DIDR0 = 0xFF; //Disable digital input registers on analogue inputs A0-7

  ADCSRB &= ~((0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0)); //Select free running. 13 ADC clock cycles per converson ADC sample rate 250kHz/13 = 19.23kS/s
  //ADCSRA |= (1 << ADSC); //start conversion
}

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
  init_adc();
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

float k_p = 0.1;
float k_d = -1;

float rps = 0;

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
        set_throttle_value = 15;
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

void loop() {
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
  if(millis() >= next_pid_update){
    rps = 0;
    uint16_t divider = 0;
    cli();
    uint8_t j = (waves_point-1) % WAVES_NUM;
    for(uint8_t i = 0; i < WAVES_NUM; i++){
      if(j < 0){
        j = WAVES_NUM-1;
      }
      if(waves_arr[j] > 0 and waves_arr[j] < 1000){
        uint16_t weight = WAVES_NUM-i;
        rps = rps + waves_arr[j]*weight;
        divider = divider+weight;
      }
      j = (j-1) % WAVES_NUM;
    }
    sei();
    if(rps > 0){
      rps = divider*1000 / rps;
    } else {
      rps = 0;
    }
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
    case 2:
      if(millis() >= next_pid_update){
        float error_tmp = set_throttle_value - rps;
        diff_error = error_tmp - error;
        error = error_tmp;
        
        float tmp = error * k_p + diff_error / (1000*PID_UPDATE_INTERVAL) * k_d;      // PD controller output
        
        if(tmp > MAX_STEP_CHANGE){
          tmp = MAX_STEP_CHANGE;
        } else if(tmp < -MAX_STEP_CHANGE){
          tmp = -MAX_STEP_CHANGE;
        }
        
        /*uint16_t ctrl = THROTTLE_STILL + tmp * (THROTTLE_FULL_POWER - THROTTLE_STILL);
        if(ctrl > (OCR1A + MAX_STEP_CHANGE)){
          OCR1A = OCR1A + MAX_STEP_CHANGE;
        } else if(ctrl < (OCR1A - MAX_STEP_CHANGE)){
          OCR1A = OCR1A - MAX_STEP_CHANGE;
        } else {
          OCR1A = ctrl; 
        }*/

        uint16_t register_tmp = OCR1A + tmp;
        if(register_tmp > THROTTLE_FULL_POWER){
          register_tmp = THROTTLE_FULL_POWER;
        } else if(register_tmp < THROTTLE_FULL_REVERSE){
          register_tmp = THROTTLE_FULL_REVERSE;
        }
        
        OCR1A = register_tmp;        
        
        next_pid_update = next_pid_update + PID_UPDATE_INTERVAL;
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
    data_flag = false;
    //Serial.print("Long term average: ");
    //Serial.print(lt_avg);
    //Serial.print(", long term sum: ");
    //Serial.print(lt_sum);
    //Serial.print(", measurement count: ");
    //Serial.print(meas_count);
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
