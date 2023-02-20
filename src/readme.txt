Source code for Flying Propeller - A Bachelor Thesis 
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
    A) .pio, which stems from PlatformIO. PlatformIO is used when programming the microcontrollers. It runs in 
    many IDEs and hackable text editors: https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation
    B) .vscode which stems from Visual Studio code
    C) include, which is empty
    D) lib, which is empty but holds place for libraries
    E) src, which contains the code that runs on the microcontrollers
    F) test, which is empty
    G) platformio.ini which is used by PlatformIO extension
    
    E) is the only file that really matters, the rest is a byproduct of using PlatformIO. It runs a continous loop
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
Andreas Grønbech Petersen (s173901@student.dtu.dk)
as part of a Bachelor Thesis in Electrical Engineering at DTU, June 2020 
