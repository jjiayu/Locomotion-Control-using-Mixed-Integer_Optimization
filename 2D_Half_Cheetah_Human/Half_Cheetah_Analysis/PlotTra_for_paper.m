warning('off')

figure
hold on

% Gallop D MiniForces

%Stride Period 0.8, Speed 1.1
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_0.8/Speed-1.1-20191217T193626.mat')
plot(y_result,theta_result,'LineWidth',4,'DisplayName','$[v,T]=[1.1,0.8]$');

%Stride Period 1.0, Speed 1.5
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-1.5-20191217T204441.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.5,1.0]$');

%Stride Period 1.0, Speed 1.7
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-1.7-20191217T212628.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.7,1.0]$');

%Stride Period 1.2, Speed 1.1
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.2/Speed-1.1-20191217T193930.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.1,1.2]$');

ylim([-0.45,0.45])

legend('Interpreter','latex','Orientation','vertical')
xlabel('$y$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')
set(gca,'FontSize',42)
legend boxoff

hold off


% %Walking S Group B4 MiniForce
figure
hold on

%Stride Period 0.8, Speed 0.7
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_0.8/Speed-0.7-20191217T184608.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.7,0.8]$');

%Stride Period 0.8, Speed 0.5
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_0.8/Speed-0.5-20191217T181534.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,0.8]$');

%Stride Period 1.0, Speed 0.5
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-0.5-20191217T182306.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.0]$');

%Stride Period 1.2, Speed 0.5
load('/home/jiayu/Dropbox/ANYmalGaitDiscovery/10_Percent_Phaselb/BoundingBoxWidth_0.4/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.2/Speed-0.5-20191217T181236.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.2]$');

legend('Interpreter','latex','Orientation','vertical')
xlabel('$y$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')
set(gca,'FontSize',42)
legend boxoff


%ylim([-0.45,0.45])

hold off