function[val]= NextState(u1,u2,u3,u4,j1,j2,j3,j4,j5,val)

r = 0.0475; %radius of wheel 
l = 0.47/2; %wheelbase
w = 0.3/2; %track width 
 
delta_t = 0.1; %time period for 1 next state iteration

%val = positions of joints, and chassis config
%val = [1_theta 2_x 3_y 4_j1 5_j2 6_j3 7_j4 8_j5 9_u1 10_u2 11_u3 12_u4 ]

%[u1,u2,u3,u4,j1,j2,j3,j4,j5] = check_vel(u1,u2,u3,u4,j1,j2,j3,j4,j5,delta_t); %check whether speeds are exceeding limits 
[val] = run_next(r,l,w,delta_t,u1,u2,u3,u4,val,j1,j2,j3,j4,j5); % execute the euler steps and odometry
end