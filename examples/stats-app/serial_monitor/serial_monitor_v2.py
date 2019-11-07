# Monitor serial input from given port and store it into given file

import sys
import argparse
import serial
from datetime import datetime
from timeit import default_timer as timer

LINES_TO_READ = 7000

DEFAULT_FILE_NAME = "rf2xx_stats.txt"

BASEPORT = "/dev/ttyUSB"
BAUD = 112500
PARITY = serial.PARITY_NONE
STOPBIT = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

gotResponse = False

def wait_response(max_time):
    startTime = timer()
    
    while((timer() - startTime) < max_time):   #TODO potestirej se malu ta sjt
        try:
            value = ser.readline()
            if(chr(value[0]) == '*'):
                gotResponse = True
                break
        except KeyboardInterrupt:
            print("\n Keyboard interrupt!..Exiting now")
            sys.exit(1)

def send_cmd(cmd):
    try:
        ser.write((cmd + "\n").encode("ASCII"))
    except:
        print("Error writing to device!")
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
parser.add_argument("-r",
                    "--root",
                    help="set device as root of the network",
                    action="store_true")

args = parser.parse_args()

# ----------------------------------------------------------------------
# Open serial monitor
# ----------------------------------------------------------------------
if(not args.port):
    # Find port automaticly - search for ttyUSB
    for i in range(0, 12):
        try:
            port = BASEPORT + str(i)
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
file = open(filename, mode="w", encoding="UTF-8")
file.write(str(datetime.now())+"\n")
file.write("----------------------------------------------------------------------------------------------- \n")
file.write("Serial input from port:" + port + "\n")
if(args.root):
    file.write("Device is root of the DAG network! \n")
file.write("----------------------------------------------------------------------------------------------- \n")
file.close()

# ----------------------------------------------------------------------
# Set device as root of the network via serial CLI
# ----------------------------------------------------------------------
if(args.root):
    print("Set device as DAG root")
    send_cmd("r")
    
# ----------------------------------------------------------------------
# Start the app
# ----------------------------------------------------------------------
print("Send start command")
send_cmd("*")

# Wait for response ('*' character) from Vesna for 3 seconds
print("Waiting for response...")
wait_response(3)

# If device is not responding, try again
if(not gotResponse):
    print("No response -> send start cmd again...")
    send_cmd("=")
    send_cmd("*")
    wait_response(3)

if(not gotResponse):
    print("No response...please reset the device and try again")
    sys.exit(1)

# ----------------------------------------------------------------------
# Read input lines while LINES_TO_READ or until stop command '='
# ----------------------------------------------------------------------
print("Start logging serial input:") 

# Open file to append serial input to it
file = open(filename, "a")

i = 1
try:
    while(i <= LINES_TO_READ):
        # Read one line (until \n char)
        value = ser.read_until(b'\n', None)

        # --------------------------------------------------------------
        # If stop command '=' found, exit monitor
        # --------------------------------------------------------------
        if(chr(value[0]) == '='):
            print("Found stop command!..stored " + str(i) + " lines.")
            break

        # Store value into file
        file.write("[" + str(datetime.now().time())+"]: ")
        value= value.decode("UTF-8")
        file.write(str(value))

        # Update status line in terminal
        print("Line " + str(i) +"/(" + str(LINES_TO_READ) +")", end="\r")
        i += 1
    
    print("")
    print("Done!..Exiting serial monitor")

except KeyboardInterrupt:
    # -------------------------------------------------------------------
    # Send stop command '*'
    # -------------------------------------------------------------------
    print("\n Keyboard interrupt!..send stop command")
    send_cmd("=")

    # -------------------------------------------------------------------
    # Get last data ("driver statistics") before closing the monitor
    # -------------------------------------------------------------------
    while(True):
        try:
            value = ser.readline()
            if(chr(value[0]) == '='):
                break
            else:
                file.write("[" + str(datetime.now().time())+"]: ")
                value= value.decode("UTF-8")
                file.write(str(value))
        except:
            print("Error closing monitor")  
            break
    print("Exiting serial monitor")


except serial.SerialException:
    print("Error opening port!..Exiting serial monitor")

except IOError:
    print("\n Serial port disconnected!.. Exiting serial monitor")

finally:
    file.close()
    ser.close()