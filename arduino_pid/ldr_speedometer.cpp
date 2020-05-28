#include "Arduino.h"
#include "constants.h"

volatile uint32_t lt_sum = 0;
volatile uint8_t data_arr[MEAS_BUF_SIZE]={0};
volatile uint16_t data_point = 0;
volatile uint32_t last_below_avg = 0;
volatile uint32_t last_above_avg = 0;
volatile uint32_t last_rotation = 0;

volatile boolean above_avg = false;
volatile uint32_t waves_arr[WAVES_NUM]={0};
volatile uint8_t waves_point = 0;
volatile uint8_t lt_avg = 0;
volatile boolean rotation = false;
volatile uint32_t rotation_count = 0;

void init_ldr_adc() {
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

double calculate_rps(bool print_output) {
    double rps = 0;
    uint16_t divider = 0;
    cli();
    uint8_t j = (waves_point-1) % WAVES_NUM;
    //uint32_t last_rotation = waves_arr[j];
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
    if(print_output){
      j = (waves_point-1) % WAVES_NUM;
      if(j < 0){
        j = WAVES_NUM-1;
      }
      float last_rps;
      if(waves_arr[j] > 0){
        last_rps = 1000 / waves_arr[j];
      } else {
        last_rps = 0;
      }
      Serial.print("Last rotation was measured at: ");
      Serial.print(last_rotation);
      Serial.print(", with a rps of: ");
      Serial.println(last_rps);
    }
    if(rps > 0){
      rps = divider*1000 / rps;
    } else {
      rps = 0;
    }
    if(OCR1A < THROTTLE_STILL){   // We are going backwards
      rps = rps*-1;
    }
    return rps;
}

ISR(ADC_vect){ //This is our interrupt service routine
    uint8_t tmp = ADCH;
    lt_sum = lt_sum - data_arr[data_point];
    data_arr[data_point] = tmp; // Store the 8 most significant bits in the data buffer and increment the bufferIndex
    lt_sum = lt_sum + tmp;
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