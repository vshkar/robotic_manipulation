function [traj9] = TrajectoryGenerator(x_start,y_start,theta_start,x_end,y_end,theta_end)
%8 steps 
%1 : move gripper to standoff from X_start
%2 : move to grasp 
%3 close grippers: 0 -1 
%4 back to standoff_1 
%5 to standoff_2
%6 move to final config 
%7 open gripper 
%8 back to standoff 2 

method =3; %time-scaling choice for the library fn trajectory 

a = [0;pi/4;0]; %(vector representation of a pi/4 rotation about y axis)
t = VecToso3(a); % se3 representation of angular velocity 

Xstart_1 = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1]; %start of ref trajectory as given in project 
%Xstart_1 = [1 0 0 0.1992; 0 1 0 0; 0 0 1 0.7535; 0 0 0 1]; %start of ref trajectory as given in project 

rot_1 = Xstart_1(1:3,1:3); %rotation part of the transformation matrix 
rot_1 = rot_1 * MatrixExp3(t); %applying rotation about the vector stated earlier  
p_end_1 = [x_start; y_start; 0.25]; %standoff position ,xstart and ystart taken from user 
Xend_1 = [rot_1 p_end_1; 0 0 0 1] ; % %combined transformation matrix 
Tf =70; %T is time period k is kept as 1 
N = Tf/0.05; %total iterations 
gripper_state = 0; 

traj1 = trajectory(Xstart_1,Xend_1,Tf,N,method,gripper_state); %finding first trajectory 

Xstart_2= Xend_1; %stand-off to drop point 
Xend_2 = Xend_1;
Xend_2(:,4) = [x_start;y_start;0.03;1];
traj2 = trajectory(Xstart_2,Xend_2,Tf,N,method,gripper_state);
%size(traj2)
gripper_state = 1; %closing the gripper for pickup 
Tf =10;
N = Tf/0.1;

traj3 = trajectory(Xend_2,Xend_2,Tf,N,method,gripper_state); %closing the gripper onto the block

Xstart_3 = Xend_2; %going back to standoff 
Xend_3 = Xstart_2;
Tf =50;
N = Tf/0.05;
traj4 = trajectory(Xstart_3,Xend_3,Tf,N,method,gripper_state);

a2 = [0;0;-pi/2]; %(vector representation of a pi/2 rotation about z axis)
t1 = VecToso3(a2); % se3 representation of angular velocity

Xstart_4 = Xend_3;
rot_4 = Xstart_4(1:3,1:3); %taking out rotation part from T

rot_4 = MatrixExp3(t1)  * rot_4;
p_end_4 = [x_end; y_end; 0.25]; % drop standoff location 
Xend_4 = [rot_4 p_end_4; 0 0 0 1] ;
Tf =50;
N = Tf/0.05;
traj5 = trajectory(Xstart_4,Xend_4,Tf,N,method,gripper_state);

Xstart_5 = Xend_4; 
rot_5 =  rot_4;
%Xend_5 = Xend_4; 
pend_5 = [x_end; y_end; 0.05]; %drop location 
Xend_5 = [rot_5 pend_5; 0 0 0 1];
%Xend_5(:,4) = [0;-1;0.03;1];
traj6 = trajectory(Xstart_5,Xend_5,Tf,N,method,gripper_state);

gripper_state = 0; %chaging gripper state to drop the block 
Xstart_6 = Xend_5;
Xend_6 = Xend_5;
Tf =10;
N = Tf/0.1;
traj7 = trajectory(Xstart_6,Xend_6,Tf,N,method,gripper_state);

Xstart_7 = Xend_6;
Xend_7 = Xstart_5;
Tf =50;
N = Tf/0.05;

traj8 = trajectory(Xstart_7,Xend_7,Tf,N,method,gripper_state);

traj9 = [traj1;traj2;traj3;traj4;traj5;traj6;traj7;traj8];%combining all trajectories in one matrix 

csvwrite("traj.csv",traj9);  %save to csv for import in vrep

end


