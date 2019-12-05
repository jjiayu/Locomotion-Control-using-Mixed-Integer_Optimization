%--------------------------------------------------------------------------
%Plot Step Frequency v.s. Step Lengths
%--------------------------------------------------------------------------

figure()
hold on
title('Humanoid - Step Frequency v.s. Step Length')
%Find samples for making the legend
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    StepFrequencyTemp = 2*1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = StepFrequencyTemp;
    y_axisTemp = StepLengthTemp;
    
    % Collect Samples for making the Legend
    if contains(GaitNameTemp,'Walking-D') == 1
        walking_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Walking-S') == 1
        walking_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Trotting') == 1
        trotting_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Gallop') == 1
        gallop_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        bounding_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        bounding_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Pronking') == 1
        pronking_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Unknown') == 1
        unknown_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'N/A') == 1
        infeasible_sample = [x_axisTemp,y_axisTemp];
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
if exist('infeasible_sample','var') == 1
    plot(infeasible_sample(1),infeasible_sample(2),'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Infeasible"];
end

%Plot the whole mapping
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    StepFrequencyTemp = 2*1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = StepFrequencyTemp;
    y_axisTemp = StepLengthTemp;
    
    if contains(GaitNameTemp,'Walking-D') == 1
        plot(x_axisTemp,y_axisTemp,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Walking-S') == 1
        plot(x_axisTemp,y_axisTemp,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    elseif contains(GaitNameTemp,'Trotting') == 1
        plot(x_axisTemp,y_axisTemp,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    elseif contains(GaitNameTemp,'Gallop') == 1
        plot(x_axisTemp,y_axisTemp,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        plot(x_axisTemp,y_axisTemp,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        plot(x_axisTemp,y_axisTemp,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Pronking') == 1
        plot(x_axisTemp,y_axisTemp,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    elseif contains(GaitNameTemp,'Unknown') == 1
        plot(x_axisTemp,y_axisTemp,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    elseif contains(GaitNameTemp,'N/A') == 1
        plot(x_axisTemp,y_axisTemp,'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    end  
    
end

xlabel('Step Frequency (Hz)')
ylabel('Step Length (m)')
legend(legend_vector,'Location','northeastoutside')
set(gca,'FontSize',24)
hold off


%--------------------------------------------------------------------------
%Plot Cadence v.s. Step Lengths
%--------------------------------------------------------------------------
figure()
hold on
title('Humanoid - Cadence v.s. Step Length')
%Find samples for making the legend
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    CadenceTemp = 1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = CadenceTemp;
    y_axisTemp = StepLengthTemp;
    
    % Collect Samples for making the Legend
    if contains(GaitNameTemp,'Walking-D') == 1
        walking_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Walking-S') == 1
        walking_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Trotting') == 1
        trotting_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Gallop') == 1
        gallop_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        bounding_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        bounding_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Pronking') == 1
        pronking_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Unknown') == 1
        unknown_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'N/A') == 1
        infeasible_sample = [x_axisTemp,y_axisTemp];
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
if exist('infeasible_sample','var') == 1
    plot(infeasible_sample(1),infeasible_sample(2),'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Infeasible"];
end

%Plot the whole mapping
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    CadenceTemp = 1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = CadenceTemp;
    y_axisTemp = StepLengthTemp;
    
    if contains(GaitNameTemp,'Walking-D') == 1
        plot(x_axisTemp,y_axisTemp,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Walking-S') == 1
        plot(x_axisTemp,y_axisTemp,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    elseif contains(GaitNameTemp,'Trotting') == 1
        plot(x_axisTemp,y_axisTemp,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    elseif contains(GaitNameTemp,'Gallop') == 1
        plot(x_axisTemp,y_axisTemp,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        plot(x_axisTemp,y_axisTemp,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        plot(x_axisTemp,y_axisTemp,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Pronking') == 1
        plot(x_axisTemp,y_axisTemp,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    elseif contains(GaitNameTemp,'Unknown') == 1
        plot(x_axisTemp,y_axisTemp,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    elseif contains(GaitNameTemp,'N/A') == 1
        plot(x_axisTemp,y_axisTemp,'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    end  
    
end

xlabel('Cadence (Hz)')
ylabel('Step Length (m)')
legend(legend_vector,'Location','northeastoutside')
set(gca,'FontSize',24)
hold off

%--------------------------------------------------------------------------
%Plot Speed v.s. Step Lengths
%--------------------------------------------------------------------------

figure()
hold on
title('Humanoid - Speed v.s. Step Length')
%Find samples for making the legend
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = StepLengthTemp;
    
    % Collect Samples for making the Legend
    if contains(GaitNameTemp,'Walking-D') == 1
        walking_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Walking-S') == 1
        walking_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Trotting') == 1
        trotting_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Gallop') == 1
        gallop_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        bounding_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        bounding_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Pronking') == 1
        pronking_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Unknown') == 1
        unknown_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'N/A') == 1
        infeasible_sample = [x_axisTemp,y_axisTemp];
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
if exist('infeasible_sample','var') == 1
    plot(infeasible_sample(1),infeasible_sample(2),'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Infeasible"];
end

%Plot the whole mapping
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; %Step Frequency: How many heal stride per second; One stride has two steps
    StepLengthTemp = DataAnalysisResult{dataIdx}.LeftStepLength;
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = StepLengthTemp;
    
    if contains(GaitNameTemp,'Walking-D') == 1
        plot(x_axisTemp,y_axisTemp,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Walking-S') == 1
        plot(x_axisTemp,y_axisTemp,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    elseif contains(GaitNameTemp,'Trotting') == 1
        plot(x_axisTemp,y_axisTemp,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    elseif contains(GaitNameTemp,'Gallop') == 1
        plot(x_axisTemp,y_axisTemp,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        plot(x_axisTemp,y_axisTemp,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        plot(x_axisTemp,y_axisTemp,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Pronking') == 1
        plot(x_axisTemp,y_axisTemp,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    elseif contains(GaitNameTemp,'Unknown') == 1
        plot(x_axisTemp,y_axisTemp,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    elseif contains(GaitNameTemp,'N/A') == 1
        plot(x_axisTemp,y_axisTemp,'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    end  
    
end

xlabel('Speed (m/x)')
ylabel('Step Length (m)')
legend(legend_vector,'Location','northeastoutside')
set(gca,'FontSize',24)
hold off

%--------------------------------------------------------------------------
%Plot Speed v.s. Step Frequency
%--------------------------------------------------------------------------
figure()
hold on
title('Humanoid - Speed v.s. Step Frequency')
%Find samples for making the legend
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; 
    StepFrequencyTemp = 2*1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = StepFrequencyTemp;
    
    % Collect Samples for making the Legend
    if contains(GaitNameTemp,'Walking-D') == 1
        walking_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Walking-S') == 1
        walking_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Trotting') == 1
        trotting_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Gallop') == 1
        gallop_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        bounding_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        bounding_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Pronking') == 1
        pronking_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Unknown') == 1
        unknown_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'N/A') == 1
        infeasible_sample = [x_axisTemp,y_axisTemp];
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
if exist('infeasible_sample','var') == 1
    plot(infeasible_sample(1),infeasible_sample(2),'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Infeasible"];
end

%Plot the whole mapping
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; 
    StepFrequencyTemp = 2*1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = StepFrequencyTemp;
    
    if contains(GaitNameTemp,'Walking-D') == 1
        plot(x_axisTemp,y_axisTemp,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Walking-S') == 1
        plot(x_axisTemp,y_axisTemp,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    elseif contains(GaitNameTemp,'Trotting') == 1
        plot(x_axisTemp,y_axisTemp,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    elseif contains(GaitNameTemp,'Gallop') == 1
        plot(x_axisTemp,y_axisTemp,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        plot(x_axisTemp,y_axisTemp,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        plot(x_axisTemp,y_axisTemp,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Pronking') == 1
        plot(x_axisTemp,y_axisTemp,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    elseif contains(GaitNameTemp,'Unknown') == 1
        plot(x_axisTemp,y_axisTemp,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    elseif contains(GaitNameTemp,'N/A') == 1
        plot(x_axisTemp,y_axisTemp,'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    end  
    
end

xlabel('Speed (m/s)')
ylabel('Step Frequency (Hz)')
legend(legend_vector,'Location','northeastoutside')
set(gca,'FontSize',24)
hold off

%--------------------------------------------------------------------------
%Plot Speed v.s. Cadence
%--------------------------------------------------------------------------

figure()
hold on
title('Humanoid - Speed v.s. Cadence')
%Find samples for making the legend
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; 
    CadenceTemp = 1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = CadenceTemp;
    
    % Collect Samples for making the Legend
    if contains(GaitNameTemp,'Walking-D') == 1
        walking_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Walking-S') == 1
        walking_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Trotting') == 1
        trotting_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Gallop') == 1
        gallop_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        bounding_s_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        bounding_d_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Pronking') == 1
        pronking_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'Unknown') == 1
        unknown_sample = [x_axisTemp,y_axisTemp];
    elseif contains(GaitNameTemp,'N/A') == 1
        infeasible_sample = [x_axisTemp,y_axisTemp];
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
if exist('infeasible_sample','var') == 1
    plot(infeasible_sample(1),infeasible_sample(2),'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    legend_vector = [legend_vector,"Infeasible"];
end

%Plot the whole mapping
for dataIdx = 1:length(DataAnalysisResult)
    %Get Quantities
    GaitNameTemp = DataAnalysisResult{dataIdx}.DiscoveredOptimalGaitName;
    SpeedTemp = DataAnalysisResult{dataIdx}.Speed; 
    CadenceTemp = 1/DataAnalysisResult{dataIdx}.StridePeriod; %Step Frequency: How many heal stride per second; One stride has two steps
    
    x_axisTemp = SpeedTemp;
    y_axisTemp = CadenceTemp;
    
    if contains(GaitNameTemp,'Walking-D') == 1
        plot(x_axisTemp,y_axisTemp,'g-s','MarkerSize',35, 'LineWidth',2.5, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Walking-S') == 1
        plot(x_axisTemp,y_axisTemp,'c-^','MarkerSize',20, 'LineWidth',2.5,'LineStyle','none')
    elseif contains(GaitNameTemp,'Trotting') == 1
        plot(x_axisTemp,y_axisTemp,'-*','MarkerSize',30,'LineWidth',4,'LineStyle','none', 'MarkerEdgeColor','#7E2F8E')
    elseif contains(GaitNameTemp,'Gallop') == 1
        plot(x_axisTemp,y_axisTemp,'m-h','MarkerSize',30, 'LineWidth',2.5, 'LineStyle','none', 'MarkerEdgeColor','m')
    elseif contains(GaitNameTemp,'Bounding-S') == 1
        plot(x_axisTemp,y_axisTemp,'-o','MarkerSize',30,'LineWidth',4, 'MarkerEdgeColor','#A2142F','LineStyle','none')
    elseif contains(GaitNameTemp,'Bounding-D') == 1
        plot(x_axisTemp,y_axisTemp,'r-+','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
    elseif contains(GaitNameTemp,'Pronking') == 1
        plot(x_axisTemp,y_axisTemp,'-d','MarkerSize',30, 'LineWidth',4, 'MarkerEdgeColor','#EDB120', 'LineStyle','none')
    elseif contains(GaitNameTemp,'Unknown') == 1
        plot(x_axisTemp,y_axisTemp,'x','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    elseif contains(GaitNameTemp,'N/A') == 1
        plot(x_axisTemp,y_axisTemp,'v','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')
    end  
    
end

xlabel('Speed (m/s)')
ylabel('Cadence (Hz)')
legend(legend_vector,'Location','northeastoutside')
set(gca,'FontSize',24)
hold off