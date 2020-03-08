function[u1,u2,u3,u4,j1,j2,j3,j4,j5]= check_vel(u1,u2,u3,u4,j1,j2,j3,j4,j5,delta_t)
%checks to see if velocities are within limits 

limit = 30; 
limit_j = 30;     %speed limits

s_u1 = limit; %each joint speed limit is specified
s_u2 = limit;
s_u3 = limit;
s_u4 = limit;
s_j1 = limit_j;
s_j2 = limit_j;
s_j3 = limit_j;
s_j4 = limit_j;
s_j5 = limit_j;

% a series of if conditions check whether speeds are more than limit. If
% they are speed is assinged limit value

if(u1>s_u1 || u1<(-1*s_u1))
u1 =s_u1;
end

if(u2>s_u2 || u2<(-1*s_u2))
u2 =s_u2;
end

if(u3>s_u3 || u3<(-1*s_u3))
u3 =s_u3;
end

if(u4>s_u4 || u4<(-1*s_u4))
u4 =s_u4;
end

if(j1>s_j1 || j1<(-1*s_j1))
j1 =s_j1;
end

if(j2>s_j2 || j2<(-1*s_j2))
j2 =s_j2;
end


if(j3>s_j3 || j3<(-1*s_j3))
j3 =s_j3;
end

if(j4>s_j4 || j4<(-1*s_j4))
j4 =s_j4;
end
if( j5>s_j5 || j5<(-1*s_j5))
j5 =s_j5;
end

end
