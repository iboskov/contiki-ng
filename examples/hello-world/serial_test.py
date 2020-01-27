# Monitor serial input from given port and store it into given file

import sys
import argparse
import serial
from datetime import datetime
from time import sleep

LINES_TO_READ = 7000

DEFAULT_FILE_NAME = "testos.txt"

PORT = "/dev/ttyS2"
BAUD = 115200
PARITY = serial.PARITY_NONE
STOPBIT = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS


#ser = serial.Serial("/dev/ttyS2", BAUD, BYTESIZE, PARITY, STOPBIT)
#print("Serial monitor opened on port: ttyS2")

ser = serial.Serial()
ser.baudrate = BAUD
ser.port = PORT
ser.parity = PARITY
ser.stopbit = STOPBIT
ser.bytesize = BYTESIZE
ser.timeout = 5


ser.open()

ser.setDTR(False)
sleep(0.10)
ser.setDTR(True)

ser.flushInput()

ser.flush()


ser.send_break(1)



print("Start logging serial input") 


i = 1
try:
    while(i <= LINES_TO_READ):
        # value = ser.read_until(b'\n', None)
        # value = ser.readline()
        value = ser.read(13)    #read 10 bytes
        print(str(value))
        # Update status line in terminal
        print(str(i) + "\n")
        i += 1
    
    print("")
    print("Done!..Exiting serial monitor")

except KeyboardInterrupt:
    print("\n Keyboard interrupt!..Exiting serial monitor")

except serial.SerialException:
    print("Error opening port!..Exiting serial monitor")

except IOError:
    print("\n Serial port disconnected!.. Exiting serial monitor")

finally:
    ser.close()
