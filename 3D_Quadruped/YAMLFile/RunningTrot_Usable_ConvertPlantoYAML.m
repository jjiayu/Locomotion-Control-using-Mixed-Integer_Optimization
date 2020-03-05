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

%Phase 1 base trajectory with LF and RH Flying (odom)
%   Base Trajectory
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
%   LF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
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
    %fprintf(fid, ['           - time: ',num2str(timing),'\n']);
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
for knotIdx = 1:11
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
        fprintf(fid, ['           - time: ',num2str(timing),'\n']);
        %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    %fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prhx_result(knotIdx)),', ',num2str(Prhy_result(knotIdx)),', ',num2str(Prhz_result(knotIdx)),']\n']);
end

%Phase 2 base trajectory all limbs flying and base cannot be controlled (base)
%   LF Trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PlfxBase(knotIdx)),', ',num2str(PlfyBase(knotIdx)),', ',num2str(PlfzBase(knotIdx)),']\n']);
end
%   LH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PlhxBase(knotIdx)),', ',num2str(PlhyBase(knotIdx)),', ',num2str(PlhzBase(knotIdx)),']\n']);
end
%   RF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PrfxBase(knotIdx)),', ',num2str(PrfyBase(knotIdx)),', ',num2str(PrfzBase(knotIdx)),']\n']);
end
%   RH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 12:21
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PrhxBase(knotIdx)),', ',num2str(PrhyBase(knotIdx)),', ',num2str(PrhzBase(knotIdx)),']\n']);
end

%Phase 3 Move Base with LH RF Flying (odom)
%   Base Trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 22:31
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)+0.03),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
end
%   LF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 22:31
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
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
for knotIdx = 22:31
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);
end

%Phase 4 base trajectory all limbs flying and base cannot be controlled (base)
%   LF Trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PlfxBase(knotIdx)),', ',num2str(PlfyBase(knotIdx)),', ',num2str(PlfzBase(knotIdx)),']\n']);
end
%   LH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PlhxBase(knotIdx)),', ',num2str(PlhyBase(knotIdx)),', ',num2str(PlhzBase(knotIdx)),']\n']);
end
%   RF Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PrfxBase(knotIdx)),', ',num2str(PrfyBase(knotIdx)),', ',num2str(PrfzBase(knotIdx)),']\n']);
end
%   RH Trajectory
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: base\n');
fprintf(fid, '         knots:\n');
timing = 0;
for knotIdx = 32:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);   
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(PrhxBase(knotIdx)),', ',num2str(PrhyBase(knotIdx)),', ',num2str(PrhzBase(knotIdx)),']\n']);
end

















%Phase 3 only base trajectory
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
timing=0;
for knotIdx = 21:30
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
for knotIdx = 31:41
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
for knotIdx = 31:41
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
for knotIdx = 31:41
    %fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    timing = timing + TimeSeries(knotIdx)-TimeSeries(knotIdx-1);
    fprintf(fid, ['           - time: ',num2str(timing),'\n']);
    fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);
end

%Finalise Printing
fclose(fid);