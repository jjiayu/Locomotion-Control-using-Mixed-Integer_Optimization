clc;
clear;
close all;

set(0,'defaulttextinterpreter','latex')

disp('Specify the Working Folder, the degrees, the one contains all different gaits')
pause(1)
WorkDir = uigetdir();

StridePeriod_Evaluated = input('Specify the Stride Period being Evaluated (i.e. 0.8):\n');

figure

hold on

%Walking-D
load([WorkDir,'/Walking_D/StridePeriod_',num2str(StridePeriod_Evaluated),'/result_summary.mat']);
plot(result_summary.speedList,log(result_summary.costList), 'g','LineWidth',6);

%Trotting
load([WorkDir,'/Trotting/StridePeriod_',num2str(StridePeriod_Evaluated),'/result_summary.mat']);
plot(result_summary.speedList,log(result_summary.costList),'LineWidth',6, 'Color','#7E2F8E');

%Galloping
load([WorkDir,'/Galloping/StridePeriod_',num2str(StridePeriod_Evaluated),'/result_summary.mat']);
plot(result_summary.speedList,log(result_summary.costList), 'LineWidth',6,'Color','#EDB120');

%Pronking
%load([WorkDir,'/Pronking/StridePeriod_',num2str(StridePeriod_Evaluated),'/result_summary.mat']);
%plot(result_summary.speedList,log(result_summary.costList), 'LineWidth',2, 'Color','#EDB120');

%Bounding-D
load([WorkDir,'/Bounding_D/StridePeriod_',num2str(StridePeriod_Evaluated),'/result_summary.mat']);
plot(result_summary.speedList,log(result_summary.costList),'r','LineWidth',6);

xlabel('$v$ (m/s)')
xlim([0.5,2.9]);
xticks(0.5:0.4:2.9)
ylabel('$log(J)$')
legend("Walking", "Trotting", "Galloping","Bounding",'Location','northeastoutside')
%title('a) Flat Terrain with $J_1$')
grid on
set(gca,'FontSize',42)
hold off

