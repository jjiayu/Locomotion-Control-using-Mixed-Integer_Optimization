clear;
clc;

load('surrounding_samples_StridePeriod_0.8_Speed_1.6.mat')

cd ~/Dropbox/Half_Cheetah_MiniForces_with_PhaseTimeLowerBound/4Phase_StridePeriod_0.8/

warning('off')

center_sample_file = 'Speed-1.6-20190926T065740.mat';
load(center_sample_file);
center_tra_no_time = [x_result;y_result;theta_result;xdot_result;ydot_result;thetadot_result;PFx_result;PFy_result;PHx_result;PHy_result;PFxdot_result;PFydot_result;PHxdot_result;PHydot_result;FFx_result;FFy_result;FHx_result;FHy_result];
center_tra_with_time = [TimeSeries;x_result;y_result;theta_result;xdot_result;ydot_result;thetadot_result;PFx_result;PFy_result;PHx_result;PHy_result;PFxdot_result;PFydot_result;PHxdot_result;PHydot_result;FFx_result;FFy_result;FHx_result;FHy_result];


dist_no_time_list = [];
dist_with_time_list = [];
speed_sample_list = [];
tra_list_no_time = {};
tra_list_with_time = {};

for result_index = 1:size(SurroundingSamplesStridePeriod0,1)
    %SurroundingSamplesStridePeriod0(i,3)
    load(SurroundingSamplesStridePeriod0(result_index,3))
    
    tra_no_time = [x_result;y_result;theta_result;xdot_result;ydot_result;thetadot_result;PFx_result;PFy_result;PHx_result;PHy_result;PFxdot_result;PFydot_result;PHxdot_result;PHydot_result;FFx_result;FFy_result;FHx_result;FHy_result];
    tra_with_time = [TimeSeries;x_result;y_result;theta_result;xdot_result;ydot_result;thetadot_result;PFx_result;PFy_result;PHx_result;PHy_result;PFxdot_result;PFydot_result;PHxdot_result;PHydot_result;FFx_result;FFy_result;FHx_result;FHy_result];    
    
    speed_sample_list(result_index) = str2double(SurroundingSamplesStridePeriod0(result_index,2));
    
    tra_list_no_time{result_index} = tra_no_time;
    tra_list_with_time{result_index} = tra_with_time;
    
    dist_no_time = sqrt(sum((center_tra_no_time - tra_no_time).^2))
    dist_no_time_list(result_index) = dist_no_time;
    dist_with_time = sqrt(sum((center_tra_with_time - tra_with_time).^2))
    dist_with_time_list(result_index) = dist_with_time;
    
end