
clear;
clc;

%-------------------------------------------------------------------------
% Load Result File
%-------------------------------------------------------------------------
disp('Load Trajectory Grouping Results')
[MINLPtrajectoryFile,MINLPtrajectoryPath] = uigetfile();
full_MINLPTrajectory_file_path = [MINLPtrajectoryPath,MINLPtrajectoryFile];
disp(['Selected File: ',full_MINLPTrajectory_file_path]);
load(full_MINLPTrajectory_file_path)
%-------------------------------------------------------------------------
% Set-up Parameters
StridePeriod_Min = input('Decide Minimum Stride Period: \n');
StridePeriod_Max = input('Decide Maximum Stride Period: \n');
StridePeriod_res = input('Decide Resolution of the Stride Period: \n');

% StridePeriod_Min = 0.4;
% StridePeriod_Max = 1.6;
% StridePeriod_res = 0.2;

StridePeriodList = StridePeriod_Min:StridePeriod_res:StridePeriod_Max;

PlotType = input('Decide which quantity is the x-axis: \n 1-> Normalised Time \n 2-> Normalised x \n 3-> y \n');

if PlotType == 1
    % Normalised Time Series as x-axis
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','x',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','xdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','y',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','ydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','theta',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','thetadot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','FFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','FFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','FHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','FHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PFxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PFydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PHxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'NormalisedTimeSeries','PHydot',StridePeriodList)
elseif PlotType == 2
    % Normalised x as x-axis
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','x',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','xdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','y',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','ydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','theta',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','thetadot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','FFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','FFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','FHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','FHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PFxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PFydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PHxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','PHydot',StridePeriodList)
elseif PlotType == 3
    % y-axis as x-axis
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','x',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','xdot',StridePeriodList)
    %Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','y',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','ydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','theta',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','thetadot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','FFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','FFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','FHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','FHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PFx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PFy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PFxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PFydot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PHx',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PHy',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PHxdot',StridePeriodList)
    Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'y','PHydot',StridePeriodList)
end


% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','y',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'Normalised_x','ydot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','xdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','y',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','theta',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','thetadot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FHx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FHy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFxdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFydot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHxdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHydot',StridePeriodList)

% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','x',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','xdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','y',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','theta',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','thetadot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FHx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FHy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFxdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PFydot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHx',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHy',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHxdot',StridePeriodList)
% Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','PHydot',StridePeriodList)

