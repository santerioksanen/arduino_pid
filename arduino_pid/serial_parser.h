#define SERIAL_BUF_SIZE 15
#define STATE_WAITING_FOR_CMD 0
#define STATE_CMD_RECEIVED 1
#define STATE_PARSING_KEY 2
#define STATE_PARSING_NUMBER 3

class SerialParser{
    public:
        SerialParser(double *Steering, double *Throttle);
        void CheckSerial();
        void ParseSerial();    
    private:
        double *steering;
        double *throttle;
        bool received;
        char incoming_cmd[SERIAL_BUF_SIZE];
        uint8_t incoming_cmd_idx;
};