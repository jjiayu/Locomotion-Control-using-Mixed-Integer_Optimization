% Generate Data
%X: Input Vector; each sample is a row, each colum is a feature/parameter (stride period, speed);
%Y: Output Label; cells

clear;
clc;

min_Speed = input('Specify minimnum speed: \n');
max_Speed = input('Specify maximum speed: \n');
Speed_resolution = input('Specify the speed resolution: \n');
min_StridePeriod = input('Specify minimum stride period: \n');
max_StridePeriod = input('Specify maximum stride period: \n');
StridePeriod_resolution = input('Specify stride Period resolution: \n');

X = zeros(length(min_Speed:Speed_resolution:max_Speed)*length(min_StridePeriod:StridePeriod_resolution:max_StridePeriod),2); %a 2D mapping
Y = cell(length(min_Speed:Speed_resolution:max_Speed)*length(min_StridePeriod:StridePeriod_resolution:max_StridePeriod),1); % 1D output vector

index = 1;
%scan the 
for StridePeriod = min_StridePeriod:StridePeriod_resolution:max_StridePeriod
    for speed = max_Speed:Speed_resolution:min_Speed
        X(index,:) = [StridePeriod, speed];
        Y{index} = 'label';
        index = index + 1;
    end
end

%Mdl = fitcknn(X,Y,'NumNeighbors',5,'Standardize',1)