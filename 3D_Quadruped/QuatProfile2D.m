quat_w = [];
quat_x = [];
quat_y = [];
quat_z = [];

for i = 1:length(theta_result)
    
    quat_temp = rotm2quat([cos(-theta_result(i)),0,sin(-theta_result(i));0,1,0;-sin(-theta_result(i)),0,cos(-theta_result(i))]);
    quat_w = [quat_w,quat_temp(1)];
    quat_x = [quat_x,quat_temp(2)];
    quat_y = [quat_y,quat_temp(3)];
    quat_z = [quat_z,quat_temp(4)];
end

QuatProfile = [TimeSeries';...
               quat_w;...
               quat_x;...
               quat_y;...
               quat_z];
            
writematrix(QuatProfile,'Quattemp.csv')
