#include "Arduino.h"
#include "serial_parser.h"

double parse_to_double(uint8_t start_idx, char *incoming_cmd){
    char *c = incoming_cmd+start_idx;
    double number = 0;
    uint8_t dec_counter = 1;
    bool dec = false;
    bool negative = false;
    uint8_t divider_idx = 0;
    double tmp = 0;
    while(isdigit(*c) || *c == '.' || *c == '-'){
        if(*c == '-'){
            negative = true;
            c++;
            continue;
        }
        if(*c == '.'){
            dec = true;
            c++;
            continue;
        }
        if(!dec){
            number *= 10;
            number += (*c - '0');
        } else {
            tmp = (*c - '0');
            for(divider_idx = 0; divider_idx < dec_counter; divider_idx++){
                tmp /= 10;
            }
            dec_counter++;
            number += tmp;
        }
        c++;
    }
    if(negative){
        number *= -1;
    }
    return number;
}

SerialParser::SerialParser(double *Steering, double *Throttle){
    steering = Steering;
    throttle = Throttle;
    received = false;
    incoming_cmd_idx = 0;
    received = false;
}

void SerialParser::CheckSerial(){
    while(Serial.available() > 0){
        incoming_cmd[incoming_cmd_idx++] = Serial.read();
        if(incoming_cmd[incoming_cmd_idx-1] == '\n'){
            received = true;
            incoming_cmd_idx = 0;
            return;
        }
    }
}

/* Format:
    Set Steering:   SS:double(-1...1)
    Set Rps:        SR:double(-inf...inf)
*/
void SerialParser::ParseSerial(){
    if(received == false){
        return;
    }

    double numb = 0;

    switch(incoming_cmd[0]){
    case 'S':   // Set
        switch(incoming_cmd[1]){
        case 'S': // Set Steering
            if(incoming_cmd[2] != ':'){
                Serial.println("NK");
                received = false;
                return;
            }
            numb = parse_to_double(3, &incoming_cmd[0]);
            if(numb >= -1 && numb <= 1){
                Serial.println("OK");
                *steering = numb;
            } else{
                Serial.println("NK");
            }
            break;
        case 'R': // Set RPS
            if(incoming_cmd[2] != ':'){
                Serial.println("NK");
                received = false;
                return;
            }
            numb = parse_to_double(3, &incoming_cmd[0]);
            Serial.println("OK");
            *throttle = numb;
            break;
        }
        break;
    }
    received = false;
}