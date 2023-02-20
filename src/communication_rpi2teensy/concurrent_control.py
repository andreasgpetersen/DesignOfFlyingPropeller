'''
This code has been developed by Jonas Brøndum (s173952@student.dtu.dk)
and Andreas Grønbech Petersen (s173901@student.dtu.dk) as
part of a Bachelor Thesis in Electrical Engineering at DTU, June 2020
 '''
import threading
import communication_functions as cf
import time
import datetime

# For storing board numbers
board_numbers = [0, 0, 0]

# For storing values for motor control
pwm_vals =[0, 0, 0]  # turns on at about 13
wing_tilt_vals = [90, 90, 90]

# For storing data from sensors
data_string = [0, 0, 0]
current = [0, 0, 0]
voltage = [0, 0, 0]
lin_ax = [0, 0, 0]
lin_ay = [0, 0, 0]
lin_az = [0, 0, 0]
gx = [0, 0, 0]
gy = [0, 0, 0]
gz = [0, 0, 0]
roll = [0, 0, 0]
pitch = [0, 0, 0]
yaw = [0, 0, 0]
time_stamps = [0, 0, 0]

# For checking if there is new data, controlling menu and logging data
choice = ""
newchoice = False
newdata = False
filename_log = ""


# Variables for PD controller
previous_error = 0
set_point = 18 # rad/s
Kp = 1.3
Kd = 0.9
last_time = time.time()
# Feedforward was found using a linear approximation:
# func that defines voltage and rotation ratio: y=2.018*x+8.404
# x=(y-8.404)/2.018 = 0.4955401388*y - 4.164519326
feedforward = 0.4955401388
ff_offset = 4.164519326
fs = 10
sample_time = 1/fs
exitchar = False
last_input = 0

# Control loop, angular velocity, PD controller
def control_angular_velocity(now, dt):
    # Declare global for variables that need changing
    global previous_error, last_time, last_input#, integral

    # Get measured_value as average from the 3 arms
    #measured_value = sum(gz)/len(gz) # if all 3 IMUs work, use this.
    measured_value = sum(gz[1:])/len(gz[1:]) # using avg from arm 2 and 3 due to
                                             # arm1 imu dying on the last day of testing

    # Calculate components
    error = set_point - measured_value
    #integral = integral + error * dt
    derivative = (error - previous_error) / dt
    print("dt:",dt)
    print("derivative:",derivative)

    # Update values for next loop
    previous_error = error
    last_time = now
    last_input = measured_value

    # Combine to get output and return
    # This is a voltage, that needs to be transferred to a PWM signal
    return Kp*error + Kd*derivative + (feedforward * set_point - ff_offset)

# Control_loop is used for running control sequences on the drone.
# At the moment, only control of angular velocity has been implemented
def control_loop():
    # Spawn thread for checking for chars in buffer to exit
    command_thread = threading.Thread(target=getCommand)
    # Set as daemon to kill it when program ends.
    command_thread.daemon = True
    # Start thread
    command_thread.start()

    while True:
        # Check for break command:
        if exitchar is True:
            break
        if command_thread.isAlive() is False:
            command_thread = threading.Thread(target=getCommand)
            command_thread.daemon = True
            command_thread.start()

        # Get time
        now = time.time()
        dt = now-last_time

        # run control loops if sample time is reached
        if sample_time <= dt:
            # Get data from sensors
            threads_read_sensor_data()

            # Log it
            log_sensor_data()

            # Get the voltage for the motors
            voltage = control_angular_velocity(now, dt)

            # Convert to PWM
            pwm = vol2pwm(voltage)
            pwm_vals[0] = pwm
            pwm_vals[1] = pwm
            pwm_vals[2] = pwm
            # Write it
            threads_write_pwm_propeller()

            # For debugging
            print("Voltage:",voltage)
            print("Angular vel:",sum(gz[1:])/len(gz[1:]))#sum(gz)/len(gz))
            print("")

# Used for checking input buffer when program is running
# called as seperate thread to not stall program
def getCommand():
    global exitchar, set_point, wing_tilt_vals
    # Get input
    char = input()
    # If it contains anything:
    if len(char) > 0:

        # sp is for changing setpoint of the PD controller while loop is running
        if char[:2] == 'sp':
            tmp = int(char[3:])
            set_point = tmp
        # a is for changing the wing tilt while the loop is running
        elif char[0] == 'a':
            # Extract value
            tmp = int(char[2:])
            # Update arrays
            wing_tilt_vals[0] = tmp
            wing_tilt_vals[1] = tmp
            wing_tilt_vals[2] = tmp
            # Write new angle
            threads_write_wing_tilt()

        # everything else is used as exit chars
        else:
            exitchar = True



# Converts a voltage level to a PWM level
def vol2pwm(vol):
    # Calculate current max in terms of voltage available on the drone
    maxvol = sum(voltage)/len(voltage)
    # If some error happend, set it to standard 11 volts
    if maxvol <= 1:
        maxvol = 11
    # Calculate pwm level
    pwm = int((vol/maxvol) * 100)
    # If unuseable result, set it to something useful
    if pwm>100:
        pwm = 100
    if pwm < 1:
        pwm = 0
    return pwm




# Get board number to correlate Serial object and arm number
# index is integer referring to whcih serUSB object
def get_board_number_wrapper(serUSB, index):
    global board_numbers

    raw = cf.getBoardNumber(serUSB)
    number = raw.decode()

    if len(number) < 1:
        # if len > 0 is false, try again
        get_board_number_wrapper(serUSB, index)
    try:
        number = int(number[0])
        board_numbers[index] = number

    except:
        print("Error in getting boardnumber, trying again")
        get_board_number_wrapper(serUSB, index)



# Get sensor data from 1 Teensy and put it into correct arrays
def get_sensor_data_wrapper(serUSB, i):
    #global sensor_data
    global current, voltage, lin_ax, lin_ay, lin_az, gx, gy, gz, roll, pitch ,yaw

    # getting data and putting it into sensor_data array for temp storage
    raw = cf.getAllSensorData(serUSB)

    # convert from byte to str
    datastr = raw.decode()

    # put it into correct array, used when logging
    data_string[i] = datastr

    # put data into correct arrays with float format
    # Format I U lin_ax lin_ay lin_az gx gy gz roll pitch yaw
    if len(datastr) > 0:
        data = datastr.split(" ")
        if len(data) > 10:
            time_stamps[i] = time.time() - start_time
            current[i] = float(data[0])
            voltage[i] = float(data[1])
            lin_ax[i] = float(data[2])
            lin_ay[i] = float(data[3])
            lin_az[i] = float(data[4])
            gx[i] = float(data[5])
            gy[i] = float(data[6])
            gz[i] = float(data[7])
            roll[i] = float(data[8])
            pitch[i] = float(data[9])
            yaw[i] = float(data[10])
        else:
            print("Error: Length of read data was not > 10")
            return -1

    else:
        print("Error: Length of read data was not > 0")

# Writes pwm value from pwm_vals to Teensy
def write_pwm_propeller_wrapper(serUSB, index):
    cf.set_prop_motor_PWM(serUSB, pwm_vals[index])

# Writes angle from wing_tilt_vals to Teensy
def write_wing_tilt_wrapper(serUSB, index):
    cf.set_wing_tilt(serUSB, wing_tilt_vals[index])

# Put data from all 3 arms into 1 string and write it to logfile
def log_sensor_data():
    # Make newdata global to edit status
    global newdata

    # If there is new data ready, treat it
    if newdata is True:
        # Create 1 long string with timestamps and data, modifying array length is to remove EOL chars
        record = (str(time_stamps[0]) + " " + data_string[0][:len(data_string[0])-1] +";" +
        str(time_stamps[1]) + " " + data_string[1][:len(data_string[1])-1] + ";" +
        str(time_stamps[2]) + " " + data_string[2][:len(data_string[2])-1] + ";" + str(pwm_vals[0]) + ";" + str(wing_tilt_vals[0]) + ";" + str(set_point) + "\n")

        # Write it to the logfile
        with open(filename_log, 'a') as f:
            f.write(record)

        # Set newdata flag to False
        newdata = False

# Spawning 3 threads for getting data from each Teensy
def threads_read_sensor_data():
    # Spawn threads
    sensor1 = threading.Thread(target=get_sensor_data_wrapper, args=(cf.serUSB1, board_numbers[0]-1))
    sensor2 = threading.Thread(target=get_sensor_data_wrapper, args=(cf.serUSB2, board_numbers[1]-1))
    sensor3 = threading.Thread(target=get_sensor_data_wrapper, args=(cf.serUSB3, board_numbers[2]-1))

    # Start threads
    sensor1.start()
    sensor2.start()
    sensor3.start()

    # Force them to wait for all each other to finish
    sensor1.join()
    sensor2.join()
    sensor3.join()

    # Set newdata flag to True, because there is new data ready
    global newdata
    newdata = True


# Spawning 3 threads for writing pwm to each Teensy
def threads_write_pwm_propeller():
    # Spawn threads
    # index for correct value is found by getting board number and subtracting 1 because board numbers are 1 indexed
    pwm1 = threading.Thread(target=write_pwm_propeller_wrapper, args=(cf.serUSB1, board_numbers[0]-1))
    pwm2 = threading.Thread(target=write_pwm_propeller_wrapper, args=(cf.serUSB2, board_numbers[1]-1))
    pwm3 = threading.Thread(target=write_pwm_propeller_wrapper, args=(cf.serUSB3, board_numbers[2]-1))

    # Start threads
    pwm1.start()
    pwm2.start()
    pwm3.start()

    # Force them to wait for each other to finish
    pwm1.join()
    pwm2.join()
    pwm3.join()


# Spawning 3 threads for writing angle to each Teensy
def threads_write_wing_tilt():
    # Spawn threads
    pwm1 = threading.Thread(target=write_wing_tilt_wrapper, args=(cf.serUSB1, board_numbers[0]-1))
    pwm2 = threading.Thread(target=write_wing_tilt_wrapper, args=(cf.serUSB2, board_numbers[1]-1))
    pwm3 = threading.Thread(target=write_wing_tilt_wrapper, args=(cf.serUSB3, board_numbers[2]-1))

    # Start threads
    pwm1.start()
    pwm2.start()
    pwm3.start()

    # Force them to wait for each other
    pwm1.join()
    pwm2.join()
    pwm3.join()


# Spawning 3 threads for getting data from each Teensy
def threads_get_board_numbers():
    # Spawn threads
    sensor1 = threading.Thread(target=get_board_number_wrapper, args=(cf.serUSB1, 0))
    sensor2 = threading.Thread(target=get_board_number_wrapper, args=(cf.serUSB2, 1))
    sensor3 = threading.Thread(target=get_board_number_wrapper, args=(cf.serUSB3, 2))

    # Start threads
    sensor1.start()
    sensor2.start()
    sensor3.start()

    # Force them to wait for all each other to finish
    sensor1.join()
    sensor2.join()
    sensor3.join()


# Write PWM of 0 to each Teensy
def kill_motors():
    print("Turning off propeller motors")
    cf.set_prop_motor_PWM(cf.serUSB1, 0)
    cf.set_prop_motor_PWM(cf.serUSB2, 0)
    cf.set_prop_motor_PWM(cf.serUSB3, 0)


# Read input from terminal. Useful when used as a thread daemon
def read_choice():
    global choice, newchoice

    # Start loop for infinite tries
    while True:
        # Get input from user
        choice = input()
        try:
            # Check if first char is an int
            tmp = int(choice[0])
        except:
            # Not an int, react:
            print("Valid number, please")
            continue
        # Check range of input
        if 0 <= tmp <= 5:
            # Range of input OK, update newchoice flag to tell menuloop
            # that it has to react on new stuff
            newchoice = True
            break
        else:
            # React on invalid range
            print("Valid range, please")


# Print help menu
def helpmenu():
    print("0: For this menu")
    print("1: Get data_string")
    print("2: Write PWM, 0-100")
    print("3: Write tilt, 0-180")
    print("4: Print board number and SerUSB name")
    print("5: Quit")


# Menu for debugging, infinite loop with different commands
def menu():
    # Set newchoice global for editing
    global newchoice, pwm_vals, wing_tilt_vals

    # Print info
    print("Debugging menu.")
    print("Format is: int=val, ie. 2=20 for sending pwm of 20 ")
    helpmenu() # print rest of menu


    # Spawn thread for reading input from user
    input_thread = threading.Thread(target=read_choice)
    # Set as daemon to kill it when program ends.
    input_thread.daemon = True
    # Start thread
    input_thread.start()



    # Run infinite loop
    while True:
        # Check for new input from user
        if newchoice is False:
            # If there is no thread for reading from user, spawn 1
            if input_thread.is_alive() is False:
                input_thread = threading.Thread(target=read_choice)
                input_thread.daemon = True
                input_thread.start()
        # React on new input from user
        if newchoice is True:
            newchoice = False  # set flag to false
            command = int(choice[0]) # 1 int is the menu choice

            # See help menu
            if command == 0:
                helpmenu()

            # Print data string to terminal
            elif command == 1:
                print(data_string)

            # Write PWM to motor
            elif command == 2:
                # Convert to int from string
                tmp = int(choice[2:])
                print("Writing new pwm values")

                # Update global PWM array
                pwm_vals[0] = tmp
                pwm_vals[1] = tmp
                pwm_vals[2] = tmp

                # Write to all motors
                threads_write_pwm_propeller()

            # Write new angle
            elif command == 3:
                # Conver to int from string
                tmp = int(choice[2:])
                print("Writing new tilt values")

                # Update global tilt array
                wing_tilt_vals[0] = tmp
                wing_tilt_vals[1] = tmp
                wing_tilt_vals[2] = tmp

                # Write to all motors
                threads_write_wing_tilt()
            elif command == 4:
                print("SerUSB1 =", board_numbers[0])
                print("SerUSB2 =", board_numbers[1])
                print("SerUSB3 =", board_numbers[2])
                # Exit
            elif command == 5:
                    break
        #update_frequency = 75 # Hz
        #time.sleep(1/update_frequency)

        # Makes sure there aren't too many threads. Usually about 2 in total, but
        # had issue with program dying at 250 threads
        # Probably not nescessary any longer, issue has been fixed
        if threading.active_count() < 200:
            threads_read_sensor_data()

        # Run function to log data. Function only logs if new data has arrived,
        # otherwise it just returns
        log_sensor_data()


# Create logfile with todays date and time as name, .txt
def create_logfile():
    global filename_log
    now = datetime.datetime.now()
    now =  now.strftime("%Y-%m-%d_%H-%M-%S")
    filename_log = now + ".txt"


    with open(filename_log, 'a') as f:
        f.write(now + "\n")


def get_wingtilt_on_startup():
    print("Please choose desired tilt angle")
    while True:
        # Get input from user
        start_angle = input("0 to 180: ")
        try:
            # Check if first char is an int
            tmp = int(start_angle[0])
            wing_tilt_vals[0] = int(start_angle)
            wing_tilt_vals[1] = int(start_angle)
            wing_tilt_vals[2] = int(start_angle)
            break;
        except:
            # Not an int, react:
            print("Valid number, please")
            continue

# Starting script
if __name__ == '__main__':
    print("Staring...")

    # get start of time for runtime
    start_time = time.time()

    # Choose wing tilt angle first, to get a correct angle in log
    # in case you were trying to reuse angle from last time
    get_wingtilt_on_startup()

    threads_write_wing_tilt()
    # create logfile with todays date and time as name
    create_logfile()

    # get board board_numbers
    threads_get_board_numbers()

    ## main menu, infinite loop for controlling and debugging
    #menu()

    print("To set a new setpoint, use \"sp=val\"")
    print("To exit: Any char except \"sp\" + enter")
    control_loop()


    # make sure to shut off motors
    kill_motors()

    print("Finished.")
    print("Runtime:  --- %s seconds ---" % (time.time() - start_time))
