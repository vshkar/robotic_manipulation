function [thetalist, success,all_joint_vectors] = IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)

clc
thetalist = thetalist0;
i = 0;

all_joint_vectors = [thetalist']; %used to store all joint vectors from the iterations 

fprintf("Iteration: %d \n",i);
maxiterations = 5; %maximum number of allowable equations   
disp("Joint Vector")
disp(thetalist')
disp("SE(3) end-effector config: ")
end_effector = FKinBody(M, Blist, thetalist); %end effector configuration at given joint angles
disp(end_effector);

Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T)); %twist error
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev; % cut-off parameter. When both go low err becomes 0, thus cutting off the while loop
disp("error twist V_b")
disp(Vb')
wb_error = norm(Vb(1:3)); %magnitude of angular velocity error
vb_error = norm(Vb(4:6)); %magnitude of linear velocity error
fprintf("angular error magnitude ||omega_b||: %d \n",wb_error);
fprintf("linear error magnitude ||v_b||: %d \n",vb_error);

%similar to starter code 
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    fprintf("Iteration: %d \n",i);
    disp("Joint Vector")
    disp(thetalist')
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    disp("SE(3) end-effector config: ")
    end_effector = FKinBody(M, Blist, thetalist);
    disp(end_effector);

    disp("error twist V_b")
    disp(Vb')
    wb_error = norm(Vb(1:3));
    vb_error = norm(Vb(4:6));
    fprintf("angular error magnitude ||omega_b||: %d \n",wb_error);
    fprintf("linear error magnitude ||v_b||: %d \n",vb_error);
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    all_joint_vectors= [all_joint_vectors; thetalist']; %each iteration is appended to the storage matrix
   
end
 disp("done!")
disp("list of joint vectors =")
disp(all_joint_vectors)
success = ~ err;
csvwrite('iterates.csv',all_joint_vectors); %writes to a .csv file the joint angles
end
