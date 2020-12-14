# MPC-Controller
A custom built linear MPC controller in MATLAB

<img src="https://github.com/AJ1897/MPC-Controller/blob/main/Additional%20Materials/MPC%20Architecture.png" width="400">

## Vehicle Model Used:
Bicycle Model\ 
x_dot = v cos(theta)\
y_dot = v sin(theta)\
theta_dot = tan(Delta)* v /cl\
v_dot = 0.5 T

## Setup
-MacOS Catalina 
-MATLAB 2020a
-Automated Driving Toolbox

## Run by
Run Local_MPC.m in MATLAB

## Results
Succesful tracking of the desired trajectories.

<img src="https://github.com/AJ1897/MPC-Controller/blob/main/Additional%20Materials/X_Y_Theta_V(P%3D10%2CC%3D10).png" width="600">


