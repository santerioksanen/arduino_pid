#ifndef actuator_h
#define actuator_h

void init_servo_pwm();

struct Pin {
    uint8_t pin_number;
    volatile uint8_t *reg;
    uint8_t reg_bit;
    volatile uint16_t *counter;
};

class Servo{

    public:
        Servo(uint8_t pin, uint16_t Min_val, uint16_t Neutral_val, uint16_t Max_val);
        bool Init(uint16_t initial_value);
        void SetValue(uint16_t value);
        void SetRelative(double value);
        uint16_t GetValue();
    
    private:
        uint8_t pin_number;
        Pin p;
        uint16_t min_val, neutral_val, max_val;
};

#endif