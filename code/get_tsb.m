function[X]= get_tsb(val,M,Blist,Tbo)
%this function helps us get the end-effect config from 'val' variable 
a = val; 
thetalist = [a(4);a(5);a(6);a(7);a(8)]; %gives us joint angle values 
Toe = FKinBody(M,Blist,thetalist); %gives end-effector config wrt to point O, 
Tsb= [cos(a(1)) -sin(a(1)) 0 a(2); sin(a(1)) cos(a(1)) 0 a(3); 0  0 1 0.0963; 0 0 0 1]; %gives location of b w.r.t fixed frame s
X = Tsb*Tbo*Toe; % used to find Tse, or actual end-effector config. subscript cancellation rule used
end
