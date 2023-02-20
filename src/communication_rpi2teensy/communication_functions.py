
import serial  # important to install pyserial and not serial, as in "pip3 install pyserial"


read_timeout = 0.01  # timeout when reading from USB, mostly useless due to us using readline()

# Declaration of Serial objects, one for each of the three Teensy boards

serUSB1 = serial.Serial(
    port='/dev/ttyACM0',  # Get correct port using 'dmesg | grep tty' and find active one
    baudrate=115200,  # USB is always 12 Mbit/sec
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, # pretty sure stopbits is unused by USB
    bytesize=serial.EIGHTBITS,
    timeout=read_timeout
)
serUSB2 = serial.Serial(
    port='/dev/ttyACM1',  # Get correct port using 'dmesg | grep tty' and find active one
    baudrate=115200,  # USB is always 12 Mbit/sec
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, # pretty sure stopbits is unused by USB
    bytesize=serial.EIGHTBITS,
    timeout=read_timeout
)
serUSB3 = serial.Serial(
    port='/dev/ttyACM2',  # Get correct port using 'dmesg | grep tty' and find active one
    baudrate=115200,  # USB is always 12 Mbit/sec
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, # pretty sure stopbits is unused by USB
    bytesize=serial.EIGHTBITS,
    timeout=read_timeout
)

# Get board number used in calibration on Teensy to correctly indicate
# which arm is which when logging. Other solution is to turn the Teensys on in
# the desired order. RPi names the usb ports after which thing is correct first
def getBoardNumber(serUSB):
    # Remove stuff in buffer, maybe useless
    serUSB.flush()

    # Create string, encode and send it.
    write_string = 'q=1\r\n'
    serUSB.write(write_string.encode('utf-8'))

    # Loop for reading response
    while True:
        try:
            # Get input, stops reading after EOL char
            readOut = serUSB.readline()
            break

        except:
            print("Could not read.")
            pass
    # Return data
    return readOut


# send A=1 to teensy, which returns all data
# only send \n from Teensy, receving device is RPI, where EOL char is \n
def getAllSensorData(serUSB):
    # Remove stuff in buffer
    serUSB.flush()

    # Create string, encode and send it.
    write_string = 'A=1\r\n'
    serUSB.write(write_string.encode('utf-8'))

    # Loop for reading response
    while True:
        try:
            # Get input, stops reading after EOL char
            readOut = serUSB.readline()
            break
        except:
            print("Could not read.")
            pass
    # Return data
    return readOut


# function for writing PWM val to Teensy, from 0 to 100,
# Starts at about PWM=13, 31/3/2020
def set_prop_motor_PWM(serUSB, PWM):
    # Clear buffer
    serUSB.flush()

    # Create string
    PWM_string = 'D=' + str(PWM) + '\n'  # format needed by Teensy

    # Send it
    serUSB.write(PWM_string.encode('utf-8'))


# write wing tilt angle to Teensy
# 90 is horizontal
def set_wing_tilt(serUSB, angle):
    # Clear buffer
    serUSB.flush()

    # Create string
    PWM_string = 'c=' + str(angle) + '\n'  # format needed by Teensy

    # Send it
    serUSB.write(PWM_string.encode('utf-8'))
