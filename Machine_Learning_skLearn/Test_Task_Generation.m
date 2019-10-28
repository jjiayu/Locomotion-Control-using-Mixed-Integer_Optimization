clear;
clc;

% Generate random sample from the condition space as test task, for
% different stride period and speed with fixed terrain slope
NumTask = input('Define number of samples (i.e. 50): \n');
disp(' ')

%Defined Stride Period and Speed Ranges
min_StridePeriod = input('Input minimum stride period (i.e. 0.4s, leave only one decimal):\n');
max_StridePeriod = input('Input maximum stride period (i.e. 1.6s, leave only one decimal):\n');

disp(' ')

min_Speed = input('Input minimum speed (i.e. 0.3 m/s, leave only one decimal): \n');
max_Speed = input('Input maximum speed (i.e. 2.5 m/s, leave only one decimal): \n');
disp(' ')

%Scale stride period and speed to integers
NumDecimals = input('Decide how many decimals do we want to keep (i.e. 1, 2): \n');
disp(' ')
Resolution = input('Decide the Resolution (minimum decimal units) of stride period and speed (i.e. 0.05):\n');
disp(' ')

Scaling_Factor = 10^NumDecimals;
Scaled_Resolution = Resolution*Scaling_Factor;
Scaled_min_StridePeriod = min_StridePeriod*Scaling_Factor;
Scaled_max_StridePeriod = max_StridePeriod*Scaling_Factor;

Scaled_min_Speed = min_Speed*Scaling_Factor;
Scaled_max_Speed = max_Speed*Scaling_Factor;

% Generate Random Samples, has to be unique
% For Stride Period
rand_Scaled_StridePeriod = randperm(Scaled_max_StridePeriod - Scaled_min_StridePeriod + 1,NumTask) + Scaled_min_StridePeriod;
rand_StridePeriod = (rand_Scaled_StridePeriod - mod(rand_Scaled_StridePeriod,Scaled_Resolution))/Scaling_Factor;

rand_Scaled_Speed = randperm(Scaled_max_Speed - Scaled_min_Speed + 1,NumTask) + Scaled_min_Speed;
rand_Speed = (rand_Scaled_Speed - mod(rand_Scaled_Speed,Scaled_Resolution))/Scaling_Factor;

% Build arrays to save into csv files
TaskPair = [rand_StridePeriod',rand_Speed'];
TaskPairTable = array2table(TaskPair,'VariableNames',["StridePeriod","Speed"]);

%Plot result
plot(TaskPair(:,1),TaskPair(:,2),'o')
xlim([min_StridePeriod-0.1*min_StridePeriod, max_StridePeriod+0.1*max_StridePeriod])
ylim([min_Speed-0.1*min_Speed, max_Speed+0.1*max_Speed])

%Decide if save these task samples
Save_Flag = input('Decide if we want to save current task sampels: 1-> yes, 2-> no \n');
disp(' ')

if Save_Flag == 1 %Save file
    Terminator_Laptop_flag = input('Select Platform (which decides the method of specifying saving directory): 1-> Laptop/Desktop 2-> Terminator \n');
    if Terminator_Laptop_flag == 1 %Laptop or Desktop
        saving_directory = uigetdir;
    elseif Terminator_Laptop_flag == 2 %terminator
        saving_directory = input('Manually Define Folder Path Storing Parameter File (quote with ''): \n');
    end
end
disp(['Saving Directory: ',saving_directory]);
disp(' ')
writetable(TaskPairTable,[saving_directory,['/TestSet_TaskSamples-',datestr(datetime('now'), 30),'.csv']]);


