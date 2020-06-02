import json
import argparse
import pyserial

def main(args):
    if args.dist is None:
        dist = 1
    else:
        dist = args.dist
    
    driven_dist = 0
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    ser.flush()
    ser.write('SR:10\n')

    while abs(driven_dist) < abs(dist):
        s = ser.readline()
        j = json.loads(s)
        driven_dist = j['revolution_count']
        ser.write('SS:0\n')
    ser.write('SR:0\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dist', default=1, help="Distance to drive")
    args = parser.parse_args()
    main(args)