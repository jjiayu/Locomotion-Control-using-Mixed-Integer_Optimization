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



for knotIdx = 1:length(TimeSeries)
    %For a single Knot
    fprintf(fid, '  - step:\n');
    % Left Front (lf) - Use Front Leg Quantities
    fprintf(fid, '    - end_effector_trajectory:\n');
    fprintf(fid, '       name: LF_LEG\n');
    fprintf(fid, '       ignore_contact: true\n');
    fprintf(fid, '       ignore_for_pose_adaptation: true\n');
    fprintf(fid, '       trajectory:\n');
    fprintf(fid, '         frame: odom\n');
    fprintf(fid, '         knots:\n');
    %Select Motion from Double Support Phase
    %Arrive first Knot Using some time (1s,2s)
    if knotIdx == 1
       fprintf(fid, ['           - time: ',num2str(2.5),'\n']);
    else
       fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(Plfx_result(knotIdx)),', ',num2str(Plfy_result(knotIdx)),', ',num2str(Plfz_result(knotIdx)),']\n']);

    
    % Left Hind (lh) - Use Hind Leg Quantities
    fprintf(fid, '    - end_effector_trajectory:\n');
    fprintf(fid, '       name: LH_LEG\n');
    fprintf(fid, '       ignore_contact: true\n');
    fprintf(fid, '       ignore_for_pose_adaptation: true\n');
    fprintf(fid, '       trajectory:\n');
    fprintf(fid, '         frame: odom\n');
    fprintf(fid, '         knots:\n');
    %Select Motion from Double Support Phase
    %Arrive first Knot Using some time (1s,2s)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(2.5),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(Plhx_result(knotIdx)),', ',num2str(Plhy_result(knotIdx)),', ',num2str(Plhz_result(knotIdx)),']\n']);

    % Right Front (rf) - Use Front Leg Quantities
    fprintf(fid, '    - end_effector_trajectory:\n');
    fprintf(fid, '       name: RF_LEG\n');
    fprintf(fid, '       ignore_contact: true\n');
    fprintf(fid, '       ignore_for_pose_adaptation: true\n');
    fprintf(fid, '       trajectory:\n');
    fprintf(fid, '         frame: odom\n');
    fprintf(fid, '         knots:\n');
    %Select Motion from Double Support Phase
    %Arrive first Knot Using some time (1s,2s)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(2.5),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(Prfx_result(knotIdx)),', ',num2str(Prfy_result(knotIdx)),', ',num2str(Prfz_result(knotIdx)),']\n']);

    % Right Hind (rh) - Use Front Leg Quantities
    fprintf(fid, '    - end_effector_trajectory:\n');
    fprintf(fid, '       name: RH_LEG\n');
    fprintf(fid, '       ignore_contact: true\n');
    fprintf(fid, '       ignore_for_pose_adaptation: true\n');
    fprintf(fid, '       trajectory:\n');
    fprintf(fid, '         frame: odom\n');
    fprintf(fid, '         knots:\n');
    %Select Motion from Double Support Phase
    %Arrive first Knot Using some time (1s,2s)
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(2.5),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(Prhx_result(knotIdx)),', ',num2str(Prhy_result(knotIdx)),', ',num2str(Prhz_result(knotIdx)),']\n']);
    
    %Robot Torso
    fprintf(fid, '    - base_trajectory:\n');
    fprintf(fid, '       trajectory:\n');
    fprintf(fid, '         frame: odom\n');
    fprintf(fid, '         knots:\n');
    if knotIdx == 1
        fprintf(fid, ['           - time: ',num2str(2.5),'\n']);
    else
        fprintf(fid, ['           - time: ',num2str(TimeSeries(knotIdx)-TimeSeries(knotIdx-1)),'\n']);
    end
    fprintf(fid, ['             position:  [',num2str(x_result(knotIdx)),', ',num2str(y_result(knotIdx)),', ',num2str(z_result(knotIdx)),']\n']);
    fprintf(fid, ['             orientation: [','deg(',num2str(phi_result(knotIdx)/pi*180),')',', ','deg(',num2str(theta_result(knotIdx)/pi*180),')',', ','deg(',num2str(psi_result(knotIdx)/pi*180),')',']','\n']);
    
    %A gap
    fprintf(fid, '\n');
end


%Finalise Printing
fclose(fid);