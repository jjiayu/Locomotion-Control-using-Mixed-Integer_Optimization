clear;
clc;

directory = uigetdir(pwd, 'Select Working Directory');

warning('off','all');

diary off
ResultCleanLog = strcat('FindOptimalGaitLog-', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary([directory, '/', ResultCleanLog]);

disp(['Working Directory: ', directory])

minSpeed = input('Specify MINIMUM Speed along X-axis: \n');

maxSpeed = input('Specify MAXIMUM Speed along X-axis: \n');

SpeedResolution = input('Specify Resolution for Scanning the Speed along X-axis: \n');

SpeedList = minSpeed:SpeedResolution:maxSpeed;

for SpeedIdx = 1:length(SpeedList)
    
    speed = SpeedList(SpeedIdx);
    
    Files = dir(fullfile(directory,['Speed-',num2str(speed),'-*.mat']));
    
    gait_results = {};
    cost_results = [];
    
    for fileIdx = 1:length(Files)
    
        load([directory,'/',Files(fileIdx).name]);
        
        gait_results = {gait_results{:},gait};
        
        cost_results = [cost_results,result_cost];
        
    end
    
    [min_cost,min_cost_Idx] = min(cost_results);
    
    disp(['For Speed ', num2str(speed), ' m/s'])
    disp('The Optimal Gait is:')
    gait_results{min_cost_Idx}
    disp(['Corresponding FileName: ', Files(min_cost_Idx).name])
    disp(['Optimal Cost: ',num2str(min_cost)])
    disp(' ')
    
end

diary off