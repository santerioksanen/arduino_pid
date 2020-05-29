#ifndef ldr_speedometer_h
#define ldr_speedometer_h

void init_ldr_adc();
void run_measurements();
double calculate_rps(bool print_output);

extern uint32_t lt_sum;
extern uint8_t data_arr[MEAS_BUF_SIZE];
extern uint16_t data_point;
extern uint32_t last_below_avg;
extern uint32_t last_above_avg;
extern uint32_t last_rotation;

extern boolean above_avg;
extern uint32_t waves_arr[WAVES_NUM];
extern uint8_t waves_point;
extern uint8_t lt_avg;
extern boolean rotation;
extern uint32_t rotation_count;


#endif