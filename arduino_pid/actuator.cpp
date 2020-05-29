#include "Arduino.h"
#include "constants.h"
#include "actuator.h"

void init_servo_pwm() {
    // Set PB1 to be an output (Pin9 Arduino UNO)
    //DDRB |= (1 << PB1);

    // Clear Timer/Counter Control Registers
    TCCR1A = 0;
    TCCR1B = 0;

    // Set inverting mode
    TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
    
    TCCR1B = (1 << WGM13) | (1 << CS11);     // Set mode to phase and frequency correct, set prescaler to 8 (16 MHz/8) = 2000 kHz 
    ICR1 = 20000;                            // Count to 20000 (20000 * 1/2000kHz) = 0.01 s = 10 ms
    //OCR1A = THROTTLE_STILL;
}

Pin pins[] = {
    {9, &DDRB, PB1, &OCR1A}
};

int16_t lookup_pin_details(uint8_t lookup_pin){
    uint8_t i=0;
    for(i=0; i<(sizeof(pins)/sizeof(pins[0])); i++){
        if(pins[i].pin_number == lookup_pin){
            return i;
        }
    }
    return -1;
}

/* Constructor
*/
Servo::Servo(uint8_t pin){
    pin_number = pin;
}

/* Init, sets desired pin as output
*/
bool Servo::Init(uint16_t initial_value){
    int16_t pin_i = lookup_pin_details(pin_number);
    if(pin_i >= 0){
        p = pins[pin_i];
        //p.reg |= (1 << p.bit);      // Set pin as output
        Serial.println("Found pin");
    } else{
        Serial.println("ERROR: Pin not found");
        return false;
    }
    *p.reg |= (1 << p.reg_bit);      // Set pin as output
    *p.counter = initial_value;
    return true;
}

void Servo::SetValue(uint16_t value){
    *p.counter = value;
}

uint16_t Servo::GetValue(){
    return *p.counter;
}