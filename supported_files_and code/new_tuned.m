disp("calling functions")
disp("feeding new block location")
disp("tuning parameters are hardcoded")
disp("block locations are [1.5,0.5,0,0.2,-1.2,(-pi/2)]")
kp =ones(6,1) * 1.1; %values of proportional gain 
ki = 0.01* ones(6,1); %values of integral gain

a= combined(1.5,0.5,0,0.2,-1.2,(-pi/2),kp,ki);