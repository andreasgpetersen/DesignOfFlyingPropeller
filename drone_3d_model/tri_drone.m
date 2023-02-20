%% Model of tri-wing drone (helicopter rotor)
% This script initializes all the variables needed for the Simscape model
% of the drone.
% The drone's physical behavior is approximated as closely as possible,
% however they are not ideal.
% The motors are modelled for hovering and as such they require a much
% smaller input voltage for a hover condition than the real-world model of
% the drone, unless fitted carefully by adjusting its Kdrag and the general
% drag of the wing.
% most things are commented/documented, however for now these are not
% implemented:
% - Gain scheduled rotation controller
% - Optimal height regulator (around 1 m as operating point)
% - pitch & roll controllers
% - position controllers
% - dynamical control switching (this is more optimum for a system like
% - this with a large area of operation (height))

% Some variables are not used either here, in the Simscape model or neither
% places, but serve as a mean of documentation.
%% parameters for control experiments
close all
clear
%
%% motorværdier
% DYS BE1806-13 2300KV 2-3S 
Kv = 2300; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
% Motor/propeller data from supplier
batVolt = 12.6; % assumed battery voltage for 3S battery - voltage measured from battery pack
rpmMax = 25300; % max RPM - assumed full battery voltage over motor
ampMax = 12; % current at max RPM in amps
powerMax = 84.4; %max output power
shaft = 0.002; %2 mm length (prop length/radius)
airDensity = 1.225; %kg/m^3
trustMax = (powerMax^2 * 2 * pi * shaft^2 *airDensity)^(1/3); % trust in [N]
% 
g = 9.80665; % m/s^2 gravity acceleration
backEMF = rpmMax/Kv; % voltage generated from rotation
RaVolt = batVolt - backEMF; % remainig voltage over resistance
Ra = RaVolt/ampMax; % (winding) resistance [Ohm]
% effect of propeller in trust and drag
Ktrust = (trustMax * g)/(rpmMax / 60 * 2 * pi); % trust i N/(rad/s)
% drag at max 
%change here
KDrag = 8 * (ampMax * Km / (rpmMax / 60 * 2 * pi)); % drag per rad/sec [Nms]
%% hover 
heightRef = 1.0;
% sample time
Ts = 0.002; % måling interval (sampletime) - sek
Ts2 = 0.03; %real time measurement and output rate
%% drone constants
mBattery = 3 * 0.125;
mBodyTotal = 0.422 + mBattery; % center modules total mass; wings and motors
% are added to the total later in this script

%wing and drone dimensions
armRadius = 0.27; %[m]
motorRadius = 0.25; % distance from center of drone [m]
armHeight = 0.02; %[m]
wheelHeight = 0.08; %[m]
wheelWidth = 0.05; %[m]
areaArmFront = armRadius*armHeight;
areaWheelFront = wheelHeight*wheelWidth;
cDFront = 0.9; % Guesstimate of frame's drag coefficient

wLength = 0.64; %m
wWidth = 0.20; %m
wHeight = 0.025; %m
wingDensity = 50; % [kg/m^3]
areaWing = 0.121; %m^2 approximated with shapes (see report)
wingMass = areaWing*wHeight*wingDensity; % [kg]
rInner = armRadius;
rOuter = armRadius + wLength;
motorMass = 0.022; % mass of one motor [kg]
propellerMass = 0.012; % scaled up to get more realistic inertia
mDrone = 1.410; % [kg] measured

%offsets for physical model appearance
%motor offsets
%the drones skeleton is 2 cm tall
offSet1 = [motorRadius 0 0.02];
offSet2 = [motorRadius*cos(2/3*pi) motorRadius*sin(2/3*pi) 0.02];
offSet3 = [motorRadius*cos(4/3*pi) motorRadius*sin(4/3*pi) 0.02];

%wing offsets
wingJointOffset1 = [armRadius 0 0.01];
wingJointOffset2 = [armRadius*cos(2/3*pi) armRadius*sin(2/3*pi) 0.01];
wingJointOffset3 = [armRadius*cos(4/3*pi) armRadius*sin(4/3*pi) 0.01];
wingOffset = [wLength/2 0 0];

weight = mDrone *g;

%% Lift and Drag coefficients generated from NASA applet (Foil Sim) for wings:
% these values are only valid for the current wings attached to the drone
% (11-06-2020)
theta = [-10 -9.5 -9.0 -8.5 -8.0 -7.5 -7.0 -6.5 -6.0 -5.5 -5.0 -4.5 -4.0 -3.5 -3.0 -2.5 -2.0 -1.5 -1.0 -0.5 0 0.5 1.0 1.5 2.0 2.5 3.0 3.5 4.0 4.5 5.0 5.5 6.0 6.5 7.0 7.5 8.0 8.5 9.0 9.5 10.0 10.5 11.0 11.5 12.0 12.5 13.0 13.5 14.0 14.5 15.0];
cLtable = [-1.07 -1.02 -0.97 -0.92 -0.87 -0.81 -0.76 -0.70 -0.65 -0.59 -0.54 -0.48 -0.43 -0.37 -0.31 -0.25 -0.19 -0.13 -0.07 -0.01 0.05 0.11 0.17 0.23 0.29 0.35 0.40 0.46 0.52 0.57 0.63 0.68 0.74 0.79 0.85 0.90 0.95 1.00 1.05 1.10 1.15 1.20 1.25 1.29 1.32 1.36 1.38 1.41 1.43 1.44 1.45];
cDtable = [0.174 0.157 0.141 0.127 0.113 0.100 0.088 0.077 0.067 0.058 0.051 0.044 0.038 0.033 0.028 0.025 0.023 0.021 0.020 0.020 0.021 0.023 0.026 0.029 0.033 0.038 0.043 0.050 0.056 0.064 0.072 0.081 0.091 0.101 0.112 0.124 0.137 0.150 0.165 0.180 0.196 0.212 0.229 0.245 0.261 0.277 0.292 0.306 0.319 0.331 0.341];
LDratio = [-6.145 -6.486 -6.854 -7.253 -7.682 -8.141 -8.628 -9.137 -9.658 -10.174 -10.656 -11.059 -11.312 -11.320 -10.955 -10.072 -8.549 -6.351 -3.600 -0.580 2.320 4.764 6.620 7.902 8.703 9.136 9.306 9.295 9.164 8.956 8.701 8.419 8.123 7.822 7.523 7.229 6.942 6.664 6.397 6.140 5.893 5.662 5.448 5.251 5.070 4.902 4.748 4.606 4.474 4.353 4.241];
pitch_lower_limit = theta(1); %[deg]
pitch_upper_limit = theta(end); %[deg]

%drag adjusted to fit real drone (this has only been tested (and confirmed) with a wing pitch of 0 deg):
%lift needs to be adjusted as well:
for i = 1:length(cDtable)
    cDtable(i) = cDtable(i)*8.4;
end
%% Omega approximation for varying lift coefficients
N = 1000000; % <- resolution; this can be scaled up or down depending on quality <<-->> processing time 
M = 31;
w = [];
wSum = 0;
dR = wLength/N;
for j = 21:(20+M)
    for i = 1:N
        wSum = wSum + sqrt((2/3*weight)/(airDensity*cLtable(j)*areaWing))/(rInner+i*dR);
    end
    w(j-20) = wSum/(N+1);
    wSum = 0;
end

%% Plot 
% figure()
% set(gcf,'paperunits','centimeters','Paperposition',[0 0 11 7])
% plot(cLtable(21:end),w,'b')
% hold on
% grid on
% title('\omega versus c_L')
% xlabel('Amplitude')
% ylabel('\omega_{avg} [rad/s]')
% legend('\omega_{avg}')
% 
% saveas(gcf,'C:\...\directory\filename','epsc')
%% Final Calculations
%the hoverRot variable determines the required rotational velocity to lift
%the drone assuming an average velocity of the wing over its span
hoverRot = w(11); %the rotational velocity required for cL = 0.52
vMotor = hoverRot*rInner; %[m/s]

%the following calculations are faulty, but hard to otherwise calculate precisely
%as they involve more in-depth knowledge or too superficial assumptions.
%hover conditions:
thrust = 1/2*airDensity*areaWing*cDtable(29)*vMotor^2;
trustPerPropeller = thrust/3; % [N]

% hover calculation
hoverVel = trustPerPropeller / Ktrust /pi * 180.0; % rotational velocity in radians/sec
hoverRPM = hoverVel/(360) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
%hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;
%% Initial conditions
%drone deployment height
heightDeploy = 0.0; % [m]
%drone initial rotation (0 or hoverRot)
initRot = 0; %[rad/s]
%% gains (this should be readjusted to gain scheduling)
%with constant added to voltage -> small gain controller
%heightRef = 5.0;
KPheightfine = 0; %prev: 0.035
KDheightfine = 0; %prev: 9.25
KIheightfine = 0;
hoverVoltagefine = 0.35*11.9;
%needs to accelerate to omega = 15.37
%without constant added to voltage -> large gain controller
hoverVoltage = 0;
%heightRef = 50;
KPheightcoarse = 0.1;
KDheightcoarse = 15.0; 
KIheightcoarse = 5.0;

%% Simulation constants (non-necessary)
simTime = 50; %[s]
stepTime = 25; %[s]

%% Running the model
mdl = 'simulink_tri_motor_drone';
open_system(mdl)
sim(mdl);