function [] = Plot_StateSpace_Trajectories(MINLP_Result_DataBase, x_axis_name, y_axis_name, StridePeriodList)
%PLOT_STATESPACE_TRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

    %Redesign some variables in the MINLP_Result_DataBase
    for idx = 1:length(MINLP_Result_DataBase)
        %   Align with same length
        %       Extend with Last - 1 Knot
%         MINLP_Result_DataBase{idx}.FFx_result = [MINLP_Result_DataBase{idx}.FFx_result;MINLP_Result_DataBase{idx}.FFx_result(end)];
%         MINLP_Result_DataBase{idx}.FFy_result = [MINLP_Result_DataBase{idx}.FFy_result;MINLP_Result_DataBase{idx}.FFy_result(end)];
%         MINLP_Result_DataBase{idx}.FHx_result = [MINLP_Result_DataBase{idx}.FHx_result;MINLP_Result_DataBase{idx}.FHx_result(end)];
%         MINLP_Result_DataBase{idx}.FHy_result = [MINLP_Result_DataBase{idx}.FHy_result;MINLP_Result_DataBase{idx}.FHy_result(end)];
%         MINLP_Result_DataBase{idx}.PFxdot_result = [MINLP_Result_DataBase{idx}.PFxdot_result;MINLP_Result_DataBase{idx}.PFxdot_result(end)];
%         MINLP_Result_DataBase{idx}.PFydot_result = [MINLP_Result_DataBase{idx}.PFydot_result;MINLP_Result_DataBase{idx}.PFydot_result(end)];
%         MINLP_Result_DataBase{idx}.PHxdot_result = [MINLP_Result_DataBase{idx}.PHxdot_result;MINLP_Result_DataBase{idx}.PHxdot_result(end)];
%         MINLP_Result_DataBase{idx}.PHydot_result = [MINLP_Result_DataBase{idx}.PHydot_result;MINLP_Result_DataBase{idx}.PHydot_result(end)];
        %       More correct -> Extend with The first Knot (the state of the last knot is the same as the first knot)
        MINLP_Result_DataBase{idx}.FFx_result = [MINLP_Result_DataBase{idx}.FFx_result;MINLP_Result_DataBase{idx}.FFx_result(1)];
        MINLP_Result_DataBase{idx}.FFy_result = [MINLP_Result_DataBase{idx}.FFy_result;MINLP_Result_DataBase{idx}.FFy_result(1)];
        MINLP_Result_DataBase{idx}.FHx_result = [MINLP_Result_DataBase{idx}.FHx_result;MINLP_Result_DataBase{idx}.FHx_result(1)];
        MINLP_Result_DataBase{idx}.FHy_result = [MINLP_Result_DataBase{idx}.FHy_result;MINLP_Result_DataBase{idx}.FHy_result(1)];
        MINLP_Result_DataBase{idx}.PFxdot_result = [MINLP_Result_DataBase{idx}.PFxdot_result;MINLP_Result_DataBase{idx}.PFxdot_result(1)];
        MINLP_Result_DataBase{idx}.PFydot_result = [MINLP_Result_DataBase{idx}.PFydot_result;MINLP_Result_DataBase{idx}.PFydot_result(1)];
        MINLP_Result_DataBase{idx}.PHxdot_result = [MINLP_Result_DataBase{idx}.PHxdot_result;MINLP_Result_DataBase{idx}.PHxdot_result(1)];
        MINLP_Result_DataBase{idx}.PHydot_result = [MINLP_Result_DataBase{idx}.PHydot_result;MINLP_Result_DataBase{idx}.PHydot_result(1)];
        %   Normalized time and x-axis
        MINLP_Result_DataBase{idx}.NormalisedTimeSeries_result = MINLP_Result_DataBase{idx}.TimeSeries./MINLP_Result_DataBase{idx}.TimeSeries(end);
        MINLP_Result_DataBase{idx}.Normalised_x_result = MINLP_Result_DataBase{idx}.x_result./MINLP_Result_DataBase{idx}.x_result(end);
        %   Repeat TimeSeries Result for easier inquiry in this function
        MINLP_Result_DataBase{idx}.TimeSeries_result = MINLP_Result_DataBase{idx}.TimeSeries;
        %MINLP_Result_DataBase{idx}.TimeSeries(end)
        %MINLP_Result_DataBase{idx}.x_result(end)
    end
    

    figure()
    %Find x-axis max min value for axis limits
    x_axis_array = [];
    for idx = 1:length(MINLP_Result_DataBase)
        x_axis_array = [x_axis_array;MINLP_Result_DataBase{idx}.([x_axis_name,'_result'])];
    end
    x_axis_max = max(x_axis_array) + abs(0.1*max(x_axis_array));
    x_axis_min = min(x_axis_array) - abs(0.1*min(x_axis_array));


    %Find y_axis max min value for axis limits
    y_axis_array = [];
    for idx = 1:length(MINLP_Result_DataBase)
        y_axis_array = [y_axis_array;MINLP_Result_DataBase{idx}.([y_axis_name,'_result'])];
    end
    y_axis_max = max(y_axis_array) + abs(0.1*max(y_axis_array));
    y_axis_min = min(y_axis_array) - abs(0.1*min(y_axis_array));

    for StridePeriodIdx = 1:length(StridePeriodList)
        
        subplot(3,3,StridePeriodIdx)
        hold on
        for idx = 1:length(MINLP_Result_DataBase)
            if MINLP_Result_DataBase{idx}.StridePeriod >= StridePeriodList(StridePeriodIdx) - StridePeriodList(StridePeriodIdx)*0.1 && MINLP_Result_DataBase{idx}.StridePeriod <= StridePeriodList(StridePeriodIdx) + StridePeriodList(StridePeriodIdx)*0.1
                plot(MINLP_Result_DataBase{idx}.([x_axis_name,'_result']),MINLP_Result_DataBase{idx}.([y_axis_name,'_result']),'LineWidth',2,'DisplayName',['Speed-',num2str(MINLP_Result_DataBase{idx}.Speed)]);
            end
        end

        xlim([x_axis_min,x_axis_max])
        ylim([y_axis_min,y_axis_max])
        xlabel(x_axis_name)
        ylabel(y_axis_name)
        title(['Stride Period: ', num2str(StridePeriodList(StridePeriodIdx))])
        lgd(StridePeriodIdx) = legend();

        hold off
    end
    suptitle(['State Space Trajectory, ',x_axis_name,' v.s.', y_axis_name])

    for StridePeriodIdx = 1:length(StridePeriodList)
        lgd(StridePeriodIdx).Location = 'northeastoutside';
    end

end
