clc
clear all
disp("calling functions")
disp("feeding standard object block location")
disp("tuning parameters are hardcoded")
kp =ones(6,1) * 1.1; %values of proportional gain 
ki = 0.01* ones(6,1); %values of integral gain
a= combined(1,0,0,0,-1,(-pi/2),kp,ki);