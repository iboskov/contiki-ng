# Monitor serial input from given port and store it into given file

import sys
import argparse
import serial
from datetime import datetime
from timeit import default_timer as timer

LINES_TO_READ = 10000
MAX_APP_TIME  = 1200000

DEFAULT_FILE_NAME = "rf2xx_stats.txt"

BASEPORT = "/dev/ttyS"
BAUD = 112500
PARITY = serial.PARITY_NONE
STOPBIT = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# ----------------------------------------------------------------------
# Monitor class
# ----------------------------------------------------------------------
class serial_monitor():

    def __init__(self):
        self.gotResponse = False

    
    def connect_to(self, p):
        try:
            self.port = "/dev/" + p
            self.ser = serial.Serial(self.port, BAUD, BYTESIZE, PARITY, STOPBIT, timeout=10)
            print("Serial monitor opened on port: " + self.port)
        except:
            print("Serial port not connected or in use!..Exiting now")
            sys.exit(1)


    def auto_connect(self):
        for i in range(0, 12):
            try:
                self.port = BASEPORT + str(i)
                self.ser = serial.Serial(self.port, BAUD, BYTESIZE, PARITY, STOPBIT, timeout=10)
                print("Serial monitor opened on port: " + self.port)
                break
            except:
                print("No serial port connected or all in use!..Exiting now")
                sys.exit(1)

    
    def read_line(self):
        value = self.ser.read_until(b'\n', None)
        return value


    def send_cmd(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode("ASCII"))
        except:
            print("Error writing to device!")


    def wait_response(self, max_time):
        startTime = timer()
        while((timer() - startTime) < max_time):
            try:
                value = self.ser.readline()
                if not value:
                    break     
                if(chr(value[0]) == '>'):
                    self.gotResponse = True
                    break
            except KeyboardInterrupt:
                print("\n Keyboard interrupt!..Exiting now")
                sys.exit(1)


    def prepare_file(self, filename):
        self.filename = filename
        self.file = open(filename, mode="w", encoding="UTF-8")
        self.file.write(str(datetime.now())+"\n")
        self.file.write("----------------------------------------------------------------------------------------------- \n")
        self.file.write("Serial input from port:" + monitor.port + "\n")
        if(args.root):
            self.file.write("Device is root of the DAG network! \n")
        self.file.write("----------------------------------------------------------------------------------------------- \n")
        self.file.close()

    
    def store_to_file(self, data):
        self.file.write("[" + str(datetime.now().time())+"]: ")
        data = data.decode("UTF-8")
        self.file.write(str(data))
        

    def close(self):
        self.ser.close()
        self.file.close()

monitor = serial_monitor()

# ----------------------------------------------------------------------
# Argument parser for selection output text file, port, root option,...
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
    monitor.auto_connect()
else:
    # Connect to given port
    monitor.connect_to(args.port)

# ----------------------------------------------------------------------
# Prepare output file
# ----------------------------------------------------------------------
if(not args.output):
    name = DEFAULT_FILE_NAME
    print("Storing into default file: " + name)
else:
    name = args.output
    print("Storing into: " + name)

# (optional) Write first lines into it
monitor.prepare_file(name)

# ----------------------------------------------------------------------
# Set device as root of the network via serial CLI
# ----------------------------------------------------------------------
if(args.root):
    print("Set device as DAG root")
    monitor.send_cmd("*Root")
    
# ----------------------------------------------------------------------
# Start the app
# ----------------------------------------------------------------------
print("Send start command")
monitor.send_cmd(">Start")

# Wait for response ('>' character) from Vesna for 3 seconds
print("Waiting for response...")
monitor.wait_response(3)

# If device is not responding, try again
if(not monitor.gotResponse):
    print("No response -> send start cmd again...")
    monitor.send_cmd("=End")
    monitor.send_cmd(">Start")
    monitor.wait_response(3)

if(not monitor.gotResponse):
    print("No response...please reset the device and try again")
    sys.exit(1)

# ----------------------------------------------------------------------
# Read input lines while LINES_TO_READ or until stop command '='
# ----------------------------------------------------------------------
# Get max duration of the app ("AD 1200000")
value = monitor.read_line()
if((chr(value[0]) == 'A') and (chr(value[1])== 'D')):
    MAX_APP_TIME = int(value[3:])

print("Start logging serial input (max " + str(MAX_APP_TIME/60000) + "min)") 

# Open file to append serial input to it
monitor.file = open(monitor.filename, "a")

line = 1
try:
    while(line <= LINES_TO_READ):
        # Read one line (until \n char)
        value = monitor.read_line()

        # Because of timeout setting, serial may return empty list
        if value:           
            # If stop command '=' found, exit monitor
            if(chr(value[0]) == '='):
                print("Found stop command (" + str(MAX_APP_TIME/60000) +
                " minutes has elapsed)..stored " + str(line) + " lines.")
                break

            # Store value into file
            monitor.store_to_file(value)

            # Update status line in terminal
            print("Line " + str(line) +"/(" + str(LINES_TO_READ) +")", end="\r")
            line += 1
    
    print("")
    print("Done!..Exiting serial monitor")

except KeyboardInterrupt:
    print("\n Keyboard interrupt!..send stop command")
    monitor.send_cmd("=End")

    # Get last data ("driver statistics") before closing the monitor
    while(True):
        try:
            value = monitor.read_line()
            if(chr(value[0]) == '='):
                break
            else:
                monitor.store_to_file(value)
        except:
            print("Error closing monitor")  
            break
    print("Exiting serial monitor")


except serial.SerialException:
    print("Error opening port!..Exiting serial monitor")

except IOError:
    print("\n Serial port disconnected!.. Exiting serial monitor")

# ----------------------------------------------------------------------
# Close the monitor
# ----------------------------------------------------------------------
finally:
    monitor.close()