How to use the model:
tri_drone.m is the script that initializes a lot of the constants and variables in the model itself. 
Some computations are not used in the model, but serve as a help for the model-user.


To run the simulink model, you will need the Simscape Suite (Simscape Multibody etc.).
The model has comments on some things, but a lot of the model is dedicated to just transforming different bodies such that the drone looks right.
If you need specification on any of the forces acting on the wings, it's best to look in the Wing 1 subsystem.
Also, you will have to update the file location of the 3D model of the arms, which is the drone3d.SLDPRT file. That's done by specifying the 
destination in the drone_body-block.

