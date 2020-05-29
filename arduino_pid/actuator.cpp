#include "Arduino.h"
#include "constants.h"

void init_servo_pwm() {
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