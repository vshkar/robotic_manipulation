function [line]= trajectory(Xstart,Xend,Tf,N,method,gripper_state)
timegap = Tf / (N - 1);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
 vector =  Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
 a_r_1 = vector(1:3,1:3);
 a_p_1 = vector(1:3,4);
 a_r_1= a_r_1';
 a_r_1 = a_r_1(:);
 line(i,:) = [a_r_1', a_p_1',gripper_state];

end

end


