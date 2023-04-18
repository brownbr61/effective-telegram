# Simple program to read in bytes from UART and write out to file
# Default file name is LED_values.txt
# Baud rate defaults to 115200
# SJG for CS6780 Mar2023

import time
import serial
import sys

# Functioning command:
# screen /dev/ttyUSB0 115200

if __name__ == '__main__':
    dir = "./"
    br = 115200
    pn = "/dev/ttyUSB0"
    # read_data(dir, br, pn)
    
    startTime = time.time()
    f = open(dir + str(sys.argv[1]), "w", encoding="ascii")
    # print("Trial " + startTime.strftime("%d/%m/%Y_%H:%M:%S"))

    # Configure + open serial instance
    ser = serial.Serial(port=pn, baudrate=br)

    if ser.is_open:
        print("Successfully opened port to Disco \n")
        print("Read a byte\n")
        # f.write('{}'.format(int.from_bytes(byte,"big")))
    else:
        print("Could not interface with Disco... shutting down \n")
        f.write("Could not successfully open serial communication with port " + pn + "at rate " + br)

    while ((time.time() - startTime) < float(sys.argv[2])):
        # f.write('{}, {}, {}\n'.format(time.time() - startTime,int.from_bytes(ser.read()+ser.read(),"big",signed=False),int.from_bytes(ser.read()+ser.read(),"big",signed=False)))
        f.write('{}, {}\n'.format(time.time() - startTime,int.from_bytes(ser.read(),"big",signed=False)))
    
    f.close()
    ser.close()
