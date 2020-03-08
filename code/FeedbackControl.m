function[speed Xerr running_total]= FeedbackControl( Xd, Xdnx,X,config,delta_t,val,running_total,Kp,Ki)


Xerr = se3ToVec(MatrixLog6(pinv(X) * Xd));  %the twist error, which when applied will correct the end-effector deviation 

Vd = (1/delta_t)* se3ToVec(MatrixLog6(pinv(Xd)*Xdnx)); %feedforward twist velocity 
running_total = Xerr*delta_t + running_total; % adds old and new running total 

abb = pinv(X) * Xd; 

V = Adjoint(abb)* Vd + Kp.* Xerr + Ki .*running_total; % the commanded end-effector twist, from feedback law

Blist = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1;0 -0.5076 -0.3526 -0.2176 0; 0.033 0 0 0 0; 0 0 0 0 0];

thetalist = [config(4);config(5);config(6);config(7);config(8)]; %creates a vector of joint angles 
Tbo = [ 1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];

Toe = FKinBody(M,Blist,thetalist); %finds Toe, required for base jacobian

r = 0.0475; 
l = 0.47/2;
w = 0.3/2;
F = (r/4) * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]; % %F is the pseudo-inverse of  H
a= size(F,2);
z = zeros(1,a);
F = [z;z;F;z]; %creates a 6 vector
J_base = Adjoint((pinv(Toe) * pinv(Tbo))) * F; %jacobian of the chassis or base 

J_arm = JacobianBody(Blist,thetalist); %body jacobian of end-effector, standard function

J = [J_base J_arm]; %creating the total 9 column jacobian 

speed = pinv(J) * V; %joint speeds for the earlier calulcated End Effector twist 

% this part implements joint limits
[identifier] = testjointlimits(speed,val); %returns 1 if joint values are exceeded, else 0
columns = find(identifier == 1); %finds the column number of the joint getting exceed 
J(:,columns) = 0; %marks as 0 the columns crossing bounds 

speed = pinv(J) * V; %the inverse is found again, for the new Jacobian, with zeroed out columns

end