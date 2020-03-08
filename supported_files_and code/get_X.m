function[X]= get_X(trajectory)
%this function just rearranges the trajetory in a SE3 format, to give the
%end-effector 
X = [trajectory(1,1) trajectory(1,2) trajectory(1,3)  trajectory(1,10); trajectory(1,4) trajectory(1,5) ...
    trajectory(1,6) trajectory(1,11); trajectory(1,7) trajectory(1,8) trajectory(1,9) trajectory(1,12); 0 0 0 1];

end
