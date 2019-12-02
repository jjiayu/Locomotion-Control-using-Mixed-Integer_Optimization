%Collect stride frequency, step length, etc.
clc;
clear;

%-------------------------------------------------------------------------
% Load Result File
%-------------------------------------------------------------------------
disp('Select the path where contains the xls file containing Trajectories with same group label:')
GroupTrajectoryFilePath = uigetdir();
GroupLabel = input('Specify the group label (with '', like Galloping_Group_A_MeanInitCollection): \n');
FilePostFix = input('Specify the Post Fix of the Group Labeling File (with '', like xlsx): \n');
GroupLabelsFileName = strcat(GroupLabel,'.',FilePostFix);

%-------------------------------------------------------------------------
% Load Excel File into Table
%-------------------------------------------------------------------------
TrajectoryTable = readtable([GroupTrajectoryFilePath,'/',GroupLabelsFileName]);

%-------------------------------------------------------------------------
% Extract Trajectories
%-------------------------------------------------------------------------
MINLP_Result_DataBase = {};
for loopIdx = 1:size(TrajectoryTable)
    disp(['Stride Period: ',num2str(TrajectoryTable.StridePeriod(loopIdx)), '  Speed: ',num2str(TrajectoryTable.Speed(loopIdx))]);
    % load result file
    MINLP_OptResult = load(TrajectoryTable.ExpFilePath{loopIdx},...
                           'TimeSeries',...
                           'x_result',   'y_result',   'theta_result',...
                           'xdot_result','ydot_result','thetadot_result',...
                           'PFx_result', 'PFy_result', 'PFxdot_result',   'PFydot_result',...
                           'PHx_result', 'PHy_result', 'PHxdot_result',   'PHydot_result',...
                           'FFx_result', 'FFy_result',...
                           'FHx_result', 'FHy_result',...
                           'PhaseSwitchingTime',... %Note, Phase Switching Time Start from a constant zero
                           'CF_result',   'CH_result',...
                           'PFcenterX_result_world','PFcenterY_result_world',...
                           'PHcenterX_result_world','PHcenterY_result_world',...
                           'result_cost');
     MINLP_OptResult.PhaseSwitchingTime = MINLP_OptResult.PhaseSwitchingTime(2:end);
     % Save Result labels
     MINLP_OptResult.StridePeriod = TrajectoryTable.StridePeriod(loopIdx);
     MINLP_OptResult.Speed = TrajectoryTable.Speed(loopIdx);
     MINLP_OptResult.ExpFileName = TrajectoryTable.ExpFileName(loopIdx);
     MINLP_OptResult.ExpFilePath = TrajectoryTable.ExpFilePath(loopIdx);
     
    %Transform the world frame quantities into robot frame
    %   Foot Step Locations
    for knotIdx = 1:length(MINLP_OptResult.theta_result)
        %Front Leg Position
        FrontLeg_Position_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),sin(MINLP_OptResult.theta_result(knotIdx));-sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PFx_result(knotIdx);MINLP_OptResult.PFy_result(knotIdx)] - [MINLP_OptResult.x_result(knotIdx);MINLP_OptResult.y_result(knotIdx)]);
        MINLP_OptResult.PFx_result_Robot(knotIdx) = FrontLeg_Position_Temp(1);
        MINLP_OptResult.PFy_result_Robot(knotIdx) = FrontLeg_Position_Temp(2);
        
        %Hind Leg Position
        HindLeg_Position_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),sin(MINLP_OptResult.theta_result(knotIdx));-sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHx_result(knotIdx);MINLP_OptResult.PHy_result(knotIdx)] - [MINLP_OptResult.x_result(knotIdx);MINLP_OptResult.y_result(knotIdx)]);
        MINLP_OptResult.PHx_result_Robot(knotIdx) = HindLeg_Position_Temp(1);
        MINLP_OptResult.PHy_result_Robot(knotIdx) = HindLeg_Position_Temp(2);
        
    end
    %   Feet Velocities
    for knotIdx = 1:length(MINLP_OptResult.theta_result) - 1      
        %Front Leg Velocity
        FrontLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PFxdot_result(knotIdx);MINLP_OptResult.PFydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PFxdot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(1);
        MINLP_OptResult.PFydot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(2);
        
        %Hind Leg Velocity
        HindLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHxdot_result(knotIdx);MINLP_OptResult.PHydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PHxdot_result_Robot(knotIdx) = HindLeg_Velo_Temp(1);
        MINLP_OptResult.PHydot_result_Robot(knotIdx) = HindLeg_Velo_Temp(2);       
    end
    
    
    %Build the trajectories        MINLP_OptResult.PFy_result_Robot(knotIdx) = FrontLeg_Position_Temp(2);
        
        %Hind Leg Position
        HindLeg_Position_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),sin(MINLP_OptResult.theta_result(knotIdx));-sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHx_result(knotIdx);MINLP_OptResult.PHy_result(knotIdx)] - [MINLP_OptResult.x_result(knotIdx);MINLP_OptResult.y_result(knotIdx)]);
        MINLP_OptResult.PHx_result_Robot(knotIdx) = HindLeg_Position_Temp(1);
        MINLP_OptResult.PHy_result_Robot(knotIdx) = HindLeg_Position_Temp(2);
        

    %   Feet Velocities
    for knotIdx = 1:length(MINLP_OptResult.theta_result) - 1      
        %Front Leg Velocity
        FrontLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PFxdot_result(knotIdx);MINLP_OptResult.PFydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PFxdot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(1);
        MINLP_OptResult.PFydot_result_Robot(knotIdx) = FrontLeg_Velo_Temp(2);
        
        %Hind Leg Velocity
        HindLeg_Velo_Temp = [cos(MINLP_OptResult.theta_result(knotIdx)),-sin(MINLP_OptResult.theta_result(knotIdx));sin(MINLP_OptResult.theta_result(knotIdx)),cos(MINLP_OptResult.theta_result(knotIdx))]*([MINLP_OptResult.PHxdot_result(knotIdx);MINLP_OptResult.PHydot_result(knotIdx)] - [MINLP_OptResult.xdot_result(knotIdx);MINLP_OptResult.ydot_result(knotIdx)]);
        MINLP_OptResult.PHxdot_result_Robot(knotIdx) = HindLeg_Velo_Temp(1);
        MINLP_OptResult.PHydot_result_Robot(knotIdx) = HindLeg_Velo_Temp(2);       
    end
    
    
    
    %Build the trajectories
    FullTrajectory_MINLP  = [MINLP_OptResult.TimeSeries;...
                             MINLP_OptResult.x_result;      MINLP_OptResult.y_result;        MINLP_OptResult.theta_result;...
                             MINLP_OptResult.xdot_result;   MINLP_OptResult.ydot_result;     MINLP_OptResult.thetadot_result;...
                             MINLP_OptResult.PFx_result;    MINLP_OptResult.PFy_result;      MINLP_OptResult.PFxdot_result;       MINLP_OptResult.PFydot_result;...
                             MINLP_OptResult.PHx_result;    MINLP_OptResult.PHy_result;      MINLP_OptResult.PHxdot_result;       MINLP_OptResult.PHydot_result;...
                             MINLP_OptResult.FFx_result;    MINLP_OptResult.FFy_result;...
                             MINLP_OptResult.FHx_result;    MINLP_OptResult.FHy_result;...
                             MINLP_OptResult.PhaseSwitchingTime]';
                                      
    FullTrajectory_MINLP_NoTimeSeries = [MINLP_OptResult.x_result;      MINLP_OptResult.y_result;        MINLP_OptResult.theta_result;...
                                         MINLP_OptResult.xdot_result;   MINLP_OptResult.ydot_result;     MINLP_OptResult.thetadot_result;...
                                         MINLP_OptResult.PFx_result;    MINLP_OptResult.PFy_result;      MINLP_OptResult.PFxdot_result;       MINLP_OptResult.PFydot_result;...
                                         MINLP_OptResult.PHx_result;    MINLP_OptResult.PHy_result;      MINLP_OptResult.PHxdot_result;       MINLP_OptResult.PHydot_result;...
                                         MINLP_OptResult.FFx_result;    MINLP_OptResult.FFy_result;...
                                         MINLP_OptResult.FHx_result;    MINLP_OptResult.FHy_result;...
                                         MINLP_OptResult.PhaseSwitchingTime]';
     
    MINLP_Result_DataBase{loopIdx} = MINLP_OptResult;
    FullTrajectoriesCollection(loopIdx,:) = FullTrajectory_MINLP';
    FullTrajectoriesCollection_NoTimeSeries(loopIdx,:) = FullTrajectory_MINLP_NoTimeSeries;

end
 
%--------------------------------------------------------------------------
% Save Data
save([GroupTrajectoryFilePath,'/',GroupLabel,'_MINLP_Trajectories','.mat'])
%--------------------------------------------------------------------------