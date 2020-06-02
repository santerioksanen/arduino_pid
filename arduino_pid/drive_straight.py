import json
import argparse
import serial
import time

update_t = 0.5

def main(args):
    if args.dist is None:
        dist = 1
    else:
        dist = float(args.dist)
    

    next_t_update = 0
    driven_dist = 0
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    ser.flush()
    ser.write(str.encode('SR:0\n'))
    ser.write(str.encode('SS:0\n'))

    while True:
        s = ser.readline().rstrip()
        try:
            j = json.loads(s)
            driven_dist_start = j['revolution_count']
            break
        except:
            pass

    while abs(driven_dist) < abs(dist):
        s = ser.readline().rstrip()
        try:
            print(s)
            j = json.loads(s)
            driven_dist = j['revolution_count'] - driven_dist_start
        except:
            pass
        
        if time.time() >= next_t_update:
            next_t_update = time.time() + update_t
            if(dist > 0):
                ser.write(str.encode('SR:40\n'))
            else:
                ser.write(str.encode('SR:-40\n'))
        
    ser.write(str.encode('SR:0\n'))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dist', default=1, help="Distance to drive")
    args = parser.parse_args()
    main(args)