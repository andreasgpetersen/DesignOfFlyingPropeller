# Design Of Flying Propeller
Bachelor thesis at DTU 2020. This thesis focuses on building, programming, and controlling a novel type of dronedesign. The drone behaves like a helicopter rotor as it rotates around its z-axis to cre-ate the lift necessary to keep it airborne.

Test flight while suspended mid air: https://www.youtube.com/watch?v=Mw67W8S0a4E

The drone has three large wings, and rotates around its own z-axis to create lift, with one Teensy Microcontroller to control each wing. 
![Photo of final drone](https://github.com/andreasgpetersen/DesignOfFlyingPropeller/blob/main/report/figures/IMG_0313.JPG)

All three Teensy boards send oirentation/location data to the Raspberry Pi which then computes the next control inputs. This runs in an infinite loop. The communication and power channels of the drones can be seen below: 
![Overview of communication and power channels](https://github.com/andreasgpetersen/DesignOfFlyingPropeller/blob/main/report/figures/analysis/Propeller_Drone_Block_Diagram.png)


This code consists of two components:

1) communication_rpi2teensy, which is Python 3 code to be run on a Raspberry Pi (only tested for 3B+).
   The three Teensy board should be connected to the RPi using USB.
   In the folder you will find:

    A) communication_functions.py
    B) concurrent_control.py

	A) is a file that contains functions for communications for B) to execute
	B) is the main script which should be executed. Either it is configured for running a control loop
	with a set reference point, or it can be used as a menu for debugging functions. At all times
	the script logs data to a file which name is the days date and time. 
	
2) wing_ctrl_v2, which is C++ code that runs on a Teensy microcontroller (only tested for Teensy 3.2)
   In the folder you will find:
    A few example output log files
    B) .pio, which stems from PlatformIO. PlatformIO is used when programming the microcontrollers. It runs in 
    many IDEs and hackable text editors: https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation
    C) .vscode which stems from Visual Studio code
    D) include, which is empty
    E) lib, which is empty but holds place for libraries
    F) src, which contains the code that runs on the microcontrollers
    G) test, which is empty
    H) platformio.ini which is used by PlatformIO extension
    
    F) is the only file that really matters, the rest is a byproduct of using PlatformIO. It runs a continous loop
    on the Teensy, in which it calculates orientation of the device and its sensor as well as issues commands
    to the propeller and servo motor

To use the code:
On the RPI, you download the scripts to a place of your choice, and run the scripts from the commandline
like you would with regular Python scripts, ie.: python3 concurrent_control.py 
Remember to install the approiate Python packages. The newest version of Python available on the RPi was used.
Consider using a virtual env to minimize errors with packages and versions. 


On the microcontroller, we used PlatformIO for programming. It runs well in Visual Studio Code.
To get started with PlatformIO follow the instructions here: https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation
Remember to install Arduino IDE and Teensyduino: https://www.pjrc.com/teensy/td_download.html

When done, import wing_ctrl_v2 to PlatformIO and build the program. If you have trouble with libraries not being
found, remember to only build the code when you're in the wing_ctrl_v2 folder.
If it succeeds (which it does at the time of writing, June 25th 2020), upload it to each Teensyboard using
the inbuilt function in PlatformIO. Remember to change the boardnumber to the correct one to use the correct
calibration.

All boards are calibrated to be used in ASTA at DTU. If used elsewhere, recalibrate. 

This code has been developed by Jonas Brøndum (s173952@student.dtu.dk) and
Andreas Grønbech Petersen (andreasgp@protonmail.com / s173901@student.dtu.dk)
as part of a Bachelor Thesis in Electrical Engineering at DTU, June 2020 
