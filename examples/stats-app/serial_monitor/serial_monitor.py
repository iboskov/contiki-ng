# Monitor serial input from given port and store it into given file

import sys
import argparse
import serial
from datetime import datetime

LINES_TO_READ = 5000

DEFAULT_FILE_NAME = "rf2xx_stats.txt"

baseport = "/dev/ttyUSB"
BAUD = 112500
PARITY = serial.PARITY_NONE
STOPBIT = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# ----------------------------------------------------------------------
# Argument parser for selection output text file - where to store data
# ----------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="Store serial input into given file.",
    formatter_class=argparse.MetavarTypeHelpFormatter
)
parser.add_argument("-o", 
                    "--output", 
                    help="select file to store serial input", 
                    type=str,
                    required=False)
parser.add_argument("-p", 
                    "--port",   
                    help="""select serial port [ttyUSBx]...if no port 
                    given, program will find it automaticly""",
                    type=str, 
                    required=False)
parser.add_argument("-s", 
                    "--skip",   
                    help="skip waiting for the start sequence", 
                    action="store_true")
parser.add_argument("-r",
                    "--root",
                    help="set device as root of the netwrok",
                    action="store_true")

args = parser.parse_args()

# ----------------------------------------------------------------------
# Open serial monitor
# ----------------------------------------------------------------------
if(not args.port):
    # Find port automaticly - search for ttyUSB
    for i in range(0, 12):
        try:
            port = baseport + str(i)
            ser = serial.Serial(port, BAUD, BYTESIZE, PARITY, STOPBIT)
            print("Serial monitor opened on port: " + port)
            break

        except:
            print("No serial port connected or all in use!..Exiting now")
            sys.exit(1)
else:
    # Connect to given port
    try:
        port = "/dev/" + args.port
        ser = serial.Serial(port, BAUD, BYTESIZE, PARITY, STOPBIT)
        print("Serial monitor opened on port: " + port)

    except:
        print("Serial port not connected or in use!..Exiting now")
        sys.exit(1)


# ----------------------------------------------------------------------
# Prepare output file
# ----------------------------------------------------------------------
if(not args.output):
    filename = DEFAULT_FILE_NAME
    print("Storing into default file: " + filename)
else:
    filename = args.output
    print("Storing into: " + filename)

# (optional) Write first lines into it
file = open(filename, "w")
file.write(str(datetime.now())+"\n")
file.write("------------------------------------------------------------------------- \n")
file.write("Serial input from port:" + port + "\n")
file.write("------------------------------------------------------------------------- \n")
file.close()

# ----------------------------------------------------------------------
# Find start sequence '*'
# ----------------------------------------------------------------------
if(not args.skip):
    print("Waiting for start sequence (reset the node)...")
    while(True):
        try:
            value = ser.readline()
            if(chr(value[0]) == '*'):
                break
        except KeyboardInterrupt:
            print("\n Keyboard interrupt!..Exiting now")
            sys.exit(1)

# ----------------------------------------------------------------------
# Set device as root of the network via serial CLI
# ----------------------------------------------------------------------
if(args.root):
    print("Set device as DAG root")
    try:
        ser.write("rpl-set-root 1 \n".encode("ASCII"))
    except:
        print("Serial write error!")
    
    # After device is set as root it responds with two lines, which we
    # do not want into log file...the loop below waits for them and 
    # then proceeds to logging
    while(True):
        try:
            value = ser.readline()
            if(chr(value[0]) == '#'):
                break
        except KeyboardInterrupt:
            print("\n Keyboard interrupt!..Exiting now")
            sys.exit(1)


print("Start logging serial input!") 

# Open file to append serial input to it
file = open(filename, "a")

# ----------------------------------------------------------------------
# Read input lines while LINES_TO_READ or until stop sequence '='
# ----------------------------------------------------------------------
i = 1
try:
    while(i <= LINES_TO_READ):
        # Read one line (until \n char)
        value = ser.read_until(b'\n', None)

        # --------------------------------------------------------------
        # If stop sequence '=' found, exit monitor
        # --------------------------------------------------------------
        if(chr(value[0]) == '='):
            print("Found stop sequence!..stored " + str(i) + " lines.")
            break

        # Store value into file
        value= value.decode("UTF-8")       
        file.write(str(value))

        # Update status line in terminal
        print("Line " + str(i) +"/(" + str(LINES_TO_READ) +")", end="\r")
        i = i +1
    
    print("")
    print("Done!")

except KeyboardInterrupt:
    print("\n Keyboard interrupt!..Exiting now")

except IOError:
    print("\n Serial port disconnected!.. Exiting now")

except serial.SerialException: 
    print("Error opening port!..Exiting now")

finally:
    file.close()
    ser.close()