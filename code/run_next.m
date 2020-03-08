function[val]= run_next(r,l,w,delta_t,u1,u2,u3,u4,val,j1,j2,j3,j4,j5)
for j = 1:(1/delta_t) %take steps based on time period (10 in this case)
     
% initial_value_vector 
speed = [j1 j2 j3 j4 j5 u1 u2 u3 u4 ];

theta_old = val(1,4:end); %joint values before euler step ( 1,2,3 are not taken as they are chassis config)
%val = [1_theta 2_x 3_y 4_j1 5_j2 6_j3 7_j4 8_j5 9_u1 10_u2 11_u3 12_u4 ]

val(1,4:end) = val(1,4:end) + speed * delta_t; %val post euler step ( 1,2,3 are not taken as they are chassis config)

theta_new = val(1,4:end); %joint values post euler step ( 1,2,3 are not taken as they are chassis config)

theta_wheel_diff= theta_new - theta_old; %delta_theta 
theta_wheel_diff = theta_wheel_diff(1,6:end); % change in wheel angles

F = (r/4) * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]; %F is the pseudo-inverse of  H
Vb = F*theta_wheel_diff'; %Twist of chassis  

%formulas and criterion from odometry text

if(Vb(1)<0.00001)  %(instead of abosulte 0, an approximation is used)
    delta_qb = [Vb(1) ; Vb(2) ;Vb(3)];
else
    delta_qb = [Vb(1); Vb(2) * sin(Vb(1)) + Vb(3) * ((cos(Vb(1)) -1)/ Vb(1)); Vb(3) * sin(Vb(1)) + Vb(2) * ((1-(cos(Vb(1))))/ Vb(1))];
end

delta_q = [1 0 0; 0 cos(val(1)) -sin(val(1)); 0 sin(val(1)) cos(val(1))] * delta_qb; %delta_q transformed to space frame 
val(1,1:3)= val(1,1:3) + delta_q'; %change in chassis values from odometry text (1 to 3 are chassis configurations 


end
