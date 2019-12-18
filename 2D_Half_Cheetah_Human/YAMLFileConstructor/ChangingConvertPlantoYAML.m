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

%Reach Initial State
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_auto:\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [-0.10, -0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [0.49143, 0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0.20, -0.12, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [-0.28922, 0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [-0.10, 0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [0.49143, -0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0.10, 0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [-0.28922, -0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0, 0.0, 0.48046]\n');
fprintf(fid, '        orientation: [deg(0), deg(6.1057), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [-0.10, -0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [0.49143, 0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0.20, -0.12, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [-0.28922, 0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [-0.10, 0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [0.49143, -0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0.10, 0.08, 0.45]\n');
fprintf(fid, '        orientation: [deg(0), deg(0), deg(0)]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - end_effector_target:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       target_position:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position:  [-0.28922, -0.1175, 0]\n');
fprintf(fid, '  - step:\n');
fprintf(fid, '    - base_target:\n');
fprintf(fid, '       target:\n');
fprintf(fid, '        frame: odom\n');
fprintf(fid, '        position: [0, 0.0, 0.48046]\n');
fprintf(fid, '        orientation: [deg(0), deg(6.1057), deg(0)]\n');

fprintf(fid, '\n');
fprintf(fid, '\n');

%Build Motion YAML file
fprintf(fid, '  - step:\n');

% Left Front (lf) - Use Front Leg Quantities
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
%Select Motion from Double Support Phase
%Arrive first Knot Using some time (1s,2s)
for knotIdx = 1:length(TimeSeries)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(PFx_result(knotIdx)),', 0.1175, ',num2str(PFy_result(knotIdx)),']\n']);
end

% Left Hind (lh) - Use Hind Leg Quantities
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: LH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
%Select Motion from Double Support Phase
%Arrive first Knot Using some time (1s,2s)
for knotIdx = 1:length(TimeSeries)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(PHx_result(knotIdx)),', 0.1175, ',num2str(PHy_result(knotIdx)),']\n']);
end

% Right Front (rf) - Use Front Leg Quantities
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RF_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
%Select Motion from Double Support Phase
%Arrive first Knot Using some time (1s,2s)
for knotIdx = 1:length(TimeSeries)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(PFx_result(knotIdx)),', -0.1175, ',num2str(PFy_result(knotIdx)),']\n']);
end

% Right Hind (rh) - Use Front Leg Quantities
fprintf(fid, '    - end_effector_trajectory:\n');
fprintf(fid, '       name: RH_LEG\n');
fprintf(fid, '       ignore_contact: false\n');
fprintf(fid, '       ignore_for_pose_adaptation: true\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
%Select Motion from Double Support Phase
%Arrive first Knot Using some time (1s,2s)
for knotIdx = 1:length(TimeSeries)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(PHx_result(knotIdx)),', -0.1175, ',num2str(PHy_result(knotIdx)),']\n']);
end

%Robot Torso
fprintf(fid, '    - base_trajectory:\n');
fprintf(fid, '       trajectory:\n');
fprintf(fid, '         frame: odom\n');
fprintf(fid, '         knots:\n');
for knotIdx = 1:length(TimeSeries)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(0),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', 0.0, ',num2str(y_result(knotIdx)),']\n']);
    fprintf(fid, ['             orientation: [deg(0), deg(',num2str(theta_result(knotIdx)/pi*180),'), deg(0)]\n']);
end
%A gap
fprintf(fid, '\n');


%Finalise Printing
fclose(fid);