function[identifier] = testjointlimits(speed,val)
% this function tests joint limits. If they are exceeding that column of
% jacobian is made 0

identifier = zeros(1,9); %create an array with default value of 0

speed2 = [speed(5) speed(6) speed(7) speed(8) speed(9) speed(1) speed(2) speed(3) speed(4)]; %rearranging speed values so that jacobian column values are matched
val(1,4:end) = val(1,4:end) + speed2;


new_val = [val(9) val(10) val(11) val(12) val(4) val(5) val(6) val(7) val(8)]; % rearraging 'val' variables so that joints are matched

%new_val(6) = joint 2
%new_val(7) = joint 3
%new_val(8) = joint 4
%the if loops decide whether joint limits are being violated. If they are
%then that column of the identifier is made high. These columns
%correpond to the joint on the 9 column jacobian
if(new_val(6)>0 || new_val(6)<-3) 
 identifier(1,6) = 1;
end

if(new_val(7)>0)
 identifier(1,7) = 1;
end
new_val(8);
if(new_val(8)>0.5)
 identifier(1,8) = 1;
end


end
