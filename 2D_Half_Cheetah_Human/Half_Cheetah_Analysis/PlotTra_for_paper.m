warning('off')

figure
hold on

% Gallop D MiniForces

%Stride Period 0.8, Speed 1.1
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_0.8/Speed-1.1-20200218T180836.mat')
%Compute Feet Trajectories
% for i = 1:length(PFx_result)
%    PF_r = [cos(theta_result(i)),-sin(theta_result(i));sin(theta_result(i)),cos(theta_result(i))]'*([PFx_result(i);PFy_result(i)]-[x_result(i);y_result(i)]);
%    PH_r = [cos(theta_result(i)),-sin(theta_result(i));sin(theta_result(i)),cos(theta_result(i))]'*([PHx_result(i);PHy_result(i)]-[x_result(i);y_result(i)]);
%    PFx_r(i) = PF_r(1);
%    PFy_r(i) = PF_r(2);
%    PHx_r(i) = PH_r(1);
%    PHy_r(i) = PH_r(2);
% end
% plot(PFx_r,PFy_r);
plot(y_result,theta_result,'LineWidth',4,'DisplayName','$[v,T]=[1.1,0.8]$');

%Stride Period 1.2, Speed 1.1
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.2/Speed-1.1-20200218T180348.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.1,1.2]$');

%Stride Period 1.0, Speed 1.5
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-1.5-20200218T183626.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.5,1.0]$');

%Stride Period 1.0, Speed 1.7
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-1.7-20200218T184706.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[1.7,1.0]$');

ylim([-0.45,0.45])
xlim([0.35,0.725])

legend('Interpreter','latex','Orientation','vertical')
xlabel('$z$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')
set(gca,'FontSize',42)
legend boxoff

hold off


% %Walking S Group B4 MiniForce
figure
hold on

%Stride Period 1.0, Speed 0.5
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1/Speed-0.5-20200218T172843.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.0]$');

%Stride Period 1.2, Speed 0.5
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.2/Speed-0.5-20200218T172635.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.2]$');

%Stride Period 1.4, Speed 0.5
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.4/Speed-0.5-20200218T172206.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.4]$');

%Stride Period 1.6, Speed 0.5
load('/home/jiayu/Dropbox/2D_ANYmal_Gait_Discovery/10_Percent_Phaselb/BoundingBoxWidth_0.5_Height_0.2/TangentialDesiredSpeed/1_MiniForces/0_Degrees/4Phases_StridePeriod_1.6/Speed-0.5-20200218T172650.mat')
plot(y_result,theta_result,'LineWidth',4, 'DisplayName','$[v,T]=[0.5,1.6]$');

legend('Interpreter','latex','Orientation','vertical')
xlabel('$z$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')
set(gca,'FontSize',42)
legend boxoff

xlim([0.35,0.64])
%ylim([-0.45,0.45])

hold off