figure()
hold on
title('Gait Mapping')
%Find samples for making the legend
for outerloop_idx = 1:size(resultMatrix,1)
    for innerloop_idx = 1:size(resultMatrix,2)
        for GaitNameLoop_Idx = 1:length(resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost)
            GaitNameTemp = resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost(GaitNameLoop_Idx);
            
            if contains(GaitNameTemp,'Walking-D') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
                walking_d_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Walking-S') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
                walking_s_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Trotting') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
                trotting_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Gallop') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
                gallop_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Bounding-S') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
                bounding_s_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Bounding-D') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
                bounding_d_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Pronking') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
                pronking_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Unknown') == 1
                %plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
                unknown_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            end   
        end
    end
end

%Plot sample for making the legend
legend_vector = [];
if exist('walking_d_sample','var') == 1
    plot(walking_d_sample(1),walking_d_sample(2),'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    legend_vector = [legend_vector,"Walking-D"];
end
if exist('walking_s_sample','var') == 1
    plot(walking_s_sample(1),walking_s_sample(2),'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    legend_vector = [legend_vector,"Walking-S"];
end
if exist('trotting_sample','var') == 1
    plot(trotting_sample(1),trotting_sample(2),'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    legend_vector = [legend_vector,"Trotting"];
end
if exist('gallop_sample','var') == 1
    plot(gallop_sample(1),gallop_sample(2),'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    legend_vector = [legend_vector,"Galloping"];
end
if exist('bounding_s_sample','var') == 1
    plot(bounding_s_sample(1),bounding_s_sample(2),'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    legend_vector = [legend_vector,"Bounding-S"];
end
if exist('bounding_d_sample','var') == 1
    plot(bounding_d_sample(1),bounding_d_sample(2),'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    legend_vector = [legend_vector,"Bounding-D"];
end
if exist('pronking_sample','var') == 1
    plot(pronking_sample(1),pronking_sample(2),'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    legend_vector = [legend_vector,"Pronking"];
end
if exist('unknown_sample','var') == 1
    plot(unknown_sample(1),unknown_sample(2),'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Unknown"];
end

%plot(walking_d_sample,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
%plot(walking_s_sample,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
%plot(trotting_sample,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
%plot(gallop_sample,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
%plot(bounding_s_sample,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
%plot(bounding_d_sample,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
%plot(pronking_sample,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
%plot(unknown_sample,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')

%Plot the whole mapping
for outerloop_idx = 1:size(resultMatrix,1)
    for innerloop_idx = 1:size(resultMatrix,2)
        for GaitNameLoop_Idx = 1:length(resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost)
            GaitNameTemp = resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost(GaitNameLoop_Idx);
            
            if contains(GaitNameTemp,'Walking-D') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
                %walking_d_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Walking-S') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
                %walking_s_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Trotting') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
                %trotting_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Gallop') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
                %gallop_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Bounding-S') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
                %bounding_s_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Bounding-D') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
                %bounding_d_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Pronking') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
                %pronking_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            elseif contains(GaitNameTemp,'Unknown') == 1
                plot(resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
                %unknown_sample = [resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2)];
            end   
        end
    end
end

%used for making title
cost_name_vector = ["1-> Minimize Force Squared (Energy Loss)",...
                    "2-> Minimize Tangential Force (Maximize Robustness)",...
                    "3-> Minimize Vibration (theta towards terrain slope, thetadot towards zero, ydot towards zero)",...
                    "4-> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)",...
                    "5-> Minimize Velocity Smoothnes with Fixed Orientatation (add orientation the same as the terrain slope)",...
                    "6-> Feet Velocity"];

xlabel('Stride Period (s)');
xlim([0.3,1.7])
ylabel('Speed (m/s)');
ylim([0.1,2.6])
title(strcat(cost_name_vector(cost_flag),' Slope:',num2str(terrain_slope_degrees), '  Degrees'));
legend(legend_vector)
set(gca,'FontSize',24)
hold off