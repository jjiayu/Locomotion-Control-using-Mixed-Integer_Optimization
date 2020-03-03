clear;
clc;
%Select Motion from Double Support Phase
disp('Select Motion Start from Double Support Phase')

ResultDirectory = uigetdir();
ResultFileName = input('Specify Trajectory Optimizaiton Result File Name, using '''' to quote the file name: \n');

%load file
warning('off')
load([ResultDirectory,'/',ResultFileName])
warning('on')

%Clear Zero
%x_result(abs(x_result)<=1e-3)=0;
%y_result(abs(y_result)<=1e-3)=0;
%z_result(abs(z_result)<=1e-3)=0;

%phi_result(abs(phi_result)<=1e-1)=0;
%theta_result(abs(theta_result)<=1e-1)=0;
%psi_result(abs(psi_result)<=1e-1)=0;

%Plfx_result(abs(Plfx_result)<=1e-3)=0;
%Plfy_result(abs(Plfy_result)<=1e-3)=0;
%Plfz_result(abs(Plfz_result)<=1e-3)=0;

%Plhx_result(abs(Plhx_result)<=1e-3)=0;
%Plhy_result(abs(Plhy_result)<=1e-3)=0;
%Plhz_result(abs(Plhz_result)<=1e-3)=0;

%Prfx_result(abs(Prfx_result)<=1e-3)=0;
%Prfy_result(abs(Prfy_result)<=1e-3)=0;
%Prfz_result(abs(Prfz_result)<=1e-3)=0;

%Prhx_result(abs(Prhx_result)<=1e-3)=0;
%Prhy_result(abs(Prhy_result)<=1e-3)=0;
%Prhz_result(abs(Prhz_result)<=1e-3)=0;

%Create folder to stor YAML motion plan
if exist([ResultDirectory,'/YAMLMotionPlan'],'dir') == 0
    mkdir([ResultDirectory,'/YAMLMotionPlan'])
end

%Make File Handle
fid = fopen([ResultDirectory,'/YAMLMotionPlan/',ResultFileName(1:end-4),'.yaml'],'wt');

%Print File Head
fprintf(fid, 'adapt_coordinates:\n');
fprintf(fid, '  - transform:\n');
fprintf(fid, '      source_frame: odom\n');
fprintf(fid, '      target_frame: odom\n');
fprintf(fid, '\n');

%Main Body
fprintf(fid, 'steps:\n');

%-------------------------------------------------------------------------
%Clycle 1
%-------------------------------------------------------------------------

%Phase 1 only base trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 1:11
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
        fprintf(fid, ['           - time: ',num2str(timing),'\n']);
        %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end

%Phase 2 Move LF RH and base
%   Base Trajectory first
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end
%   LF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Plfx_result(knotIdx)),', ',num2str(Plfy_result(knotIdx)),', ',num2str(Plfz_result(knotIdx)),']\n']);
end
%   RH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prhx_result(knotIdx)),', ',num2str(Prhy_result(knotIdx)),', ',num2str(Prhz_result(knotIdx)),']\n']);
end

%Phase 3 only base trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 22:31
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end

%Phase 4 LH RF and base
%   Base Trajectory first
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end
%   LH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 32:41
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(Plhx_result(knotIdx)),', ',num2str(Plhy_result(knotIdx)),', ',num2str(Plhz_result(knotIdx)),']\n']);
end
%   RF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Cycle 2
%--------------------------------------------------------------------------
%Phase 1 only base trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 42:51
        timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
        fprintf(fid, ['           - time: ',num2str(timing),'\n']);
        %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
        fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
        fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end

%Phase 2 Move LF RH and base
%   Base Trajectory first
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 52:61
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end
%   LF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 52:61
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Plfx_result(knotIdx)),', ',num2str(Plfy_result(knotIdx)),', ',num2str(Plfz_result(knotIdx)),']\n']);
end
%   RH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 52:61
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prhx_result(knotIdx)),', ',num2str(Prhy_result(knotIdx)),', ',num2str(Prhz_result(knotIdx)),']\n']);
end

%Phase 3 only base trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 62:71
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end

%Phase 4 LH RF and base
%   Base Trajectory first
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 72:81
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end
%   LH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 72:81
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(Plhx_result(knotIdx)),', ',num2str(Plhy_result(knotIdx)),', ',num2str(Plhz_result(knotIdx)),']\n']);
end
%   RF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 72:81
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);
end






% for knotIdx = 1:length(TimeSeries)
%     %For a single Knot
%     fprintf(fid, '  - step:\n');
%     % Left Front (lf) - Use Front Leg Quantities
%     fprintf(fid, '    - end_effector_trajectory:\n');
%     fprintf(fid, '       name: LF_LEG\n');
%     fprintf(fid, '       ignore_contact: false\n');
%     fprintf(fid, '       ignore_for_pose_adaptation: true\n');
%     fprintf(fid, '       trajectory:\n');
%     fprintf(fid, '         frame: odom\n');
%     fprintf(fid, '         knots:\n');
%     %Select Motion from Double Support Phase
%     %Arrive first Knot Using some time (1s,2s)
%     if knotIdx == 1
%        fprintf(fid, ['           - time: ',num2str(0),'\n']);
%     else
%        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
%     end
%     fprintf(fid, ['             position:  [',num2str(Plfx_result(knotIdx)),', ',num2str(Plfy_result(knotIdx)),', ',num2str(Plfz_result(knotIdx)),']\n']);
% 
%     
%     % Left Hind (lh) - Use Hind Leg Quantities
%     fprintf(fid, '    - end_effector_trajectory:\n');
%     fprintf(fid, '       name: LH_LEG\n');
%     fprintf(fid, '       ignore_contact: false\n');
%     fprintf(fid, '       ignore_for_pose_adaptation: true\n');
%     fprintf(fid, '       trajectory:\n');
%     fprintf(fid, '         frame: odom\n');
%     fprintf(fid, '         knots:\n');
%     %Select Motion from Double Support Phase
%     %Arrive first Knot Using some time (1s,2s)
%     if knotIdx == 1
%         fprintf(fid, ['           - time: ',num2str(0),'\n']);
%     else
%         fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
%     end
%     fprintf(fid, ['             position:  [',num2str(Plhx_result(knotIdx)),', ',num2str(Plhy_result(knotIdx)),', ',num2str(Plhz_result(knotIdx)),']\n']);
% 
%     % Right Front (rf) - Use Front Leg Quantities
%     fprintf(fid, '    - end_effector_trajectory:\n');
%     fprintf(fid, '       name: RF_LEG\n');
%     fprintf(fid, '       ignore_contact: false\n');
%     fprintf(fid, '       ignore_for_pose_adaptation: true\n');
%     fprintf(fid, '       trajectory:\n');
%     fprintf(fid, '         frame: odom\n');
%     fprintf(fid, '         knots:\n');
%     %Select Motion from Double Support Phase
%     %Arrive first Knot Using some time (1s,2s)
%     if knotIdx == 1
%         fprintf(fid, ['           - time: ',num2str(0),'\n']);
%     else
%         fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
%     end
%     fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);
% 
%     % Right Hind (rh) - Use Front Leg Quantities
%     fprintf(fid, '    - end_effector_trajectory:\n');
%     fprintf(fid, '       name: RH_LEG\n');
%     fprintf(fid, '       ignore_contact: false\n');
%     fprintf(fid, '       ignore_for_pose_adaptation: true\n');
%     fprintf(fid, '       trajectory:\n');
%     fprintf(fid, '         frame: odom\n');
%     fprintf(fid, '         knots:\n');
%     %Select Motion from Double Support Phase
%     %Arrive first Knot Using some time (1s,2s)
%     if knotIdx == 1
%         fprintf(fid, ['           - time: ',num2str(0),'\n']);
%     else
%         fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
%     end
%     fprintf(fid, ['             position:  [',num2str(Prhx_result(knotIdx)),', ',num2str(Prhy_result(knotIdx)),', ',num2str(Prhz_result(knotIdx)),']\n']);
%     
%     %Robot Torso
%     fprintf(fid, '    - base_trajectory:\n');
%     fprintf(fid, '       trajectory:\n');
%     fprintf(fid, '         frame: odom\n');
%     fprintf(fid, '         knots:\n');
%     if knotIdx == 1
%         fprintf(fid, ['           - time: ',num2str(0),'\n']);
%     else
%         fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
%     end
%     fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
%     fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
%     
%     %A gap
%     fprintf(fid, '\n');
% end


%Finalise Printing
fclose(fid);