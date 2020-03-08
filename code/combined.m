function[err]= best_tuned(x_start,y_start,theta_start,x_end,y_end,theta_end,kp,ki)

delta_t = 0.1; %time period 

M = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; %end-effector config at home position 
Blist = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1;0 -0.5076 -0.3526 -0.2176 0; 0.033 0 0 0 0; 0 0 0 0 0]; %blist values at home 
Tbo = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % offset of arm base from chassis, given


val = [0.2 -0.5 0.1 0 -0.8 0 -0.3 0 0 0 0 0]; %initial position

%val = [1_theta 2_x 3_y 4_j1 5_j2 6_j3 7_j4 8_j5 9_u1 10_u2 11_u3 12_u4 ],
%format of the val variable 

store2 = [val 0]; %a variable to store trajectory from next state

X=get_tsb(val,M,Blist,Tbo); %gets the actual end-effector configuration from the initial_position 

ref_trajectory = TrajectoryGenerator(x_start,y_start,theta_start,x_end,y_end,theta_end); %trajectory generator is called to get the list of all way-points

err=[]; %empty set to store twist error 
speed_store = []; %empty set to store speed values for debugging 

running_total =0; %keeps a track of Xerr * time for integral
for i =1:size(ref_trajectory)-1

Xd = get_X(ref_trajectory(i,:)); %desired end-effector config from the ith trajectory iteration 
Xdnx = get_X(ref_trajectory(i+1,:));   %next end-effector config from trajectory 

[speed Xerr running_total] = FeedbackControl(Xd, Xdnx,X,val,delta_t,val,running_total,kp,ki); %run feedback control to get speed values 

err = [err ; Xerr']; %error is stored for plotting

speed_store =[ speed_store; speed']; %speed is stored for plotting 

[val]= NextState(speed(1),speed(2),speed(3),speed(4),speed(5),speed(6),speed(7),speed(8),speed(9),val); %speed values are given to next state function to get the next state

X = get_tsb(val,M,Blist,Tbo); %actual end-effector config as per the value got from next state function.

o = [val ref_trajectory(i,13)]; %13th variable, i.e gripper conifg is taken from the ref_trajectory ( not given as part of 'val' variable )
store2 = [store2;o]; % stores the configs from next state for saving to csv

end

plot(err)
legend({'wx','wy','wz','vx','vy','vz'},'Location','northeast')
title("Xerr vs time")
xlabel("time iterations")
ylabel("value of individual components")
csvwrite("trajectory.csv",store2);  %save to csv for import in vrep
end