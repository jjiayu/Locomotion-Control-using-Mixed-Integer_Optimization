clear;
clc;

directory = uigetdir([pwd,'/LargerNumNodes_GaitSelectionResult/'], 'Select Working Directory');

warning('off','all');

diary off
ResultCleanLog = strcat('Gait-Optimization-Results', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary([directory, '/', ResultCleanLog]);

disp(['Working Directory: ', directory])

minSpeed = input('Specify MINIMUM Speed along X-axis: \n');

maxSpeed = input('Specify MAXIMUM Speed along X-axis: \n');

SpeedResolution = input('Specify Resolution for Scanning the Speed along X-axis: \n');

Query_SpeedList = minSpeed:SpeedResolution:maxSpeed;

for SpeedIdx = 1:length(Query_SpeedList)
    
    speed = Query_SpeedList(SpeedIdx);
    
    disp('===========================================================')
    disp(['For Speed: ', num2str(speed), ' m/s'])

    
    Files = dir(fullfile(directory,['Speed-',num2str(speed),'-*.mat']));
    
    gait_results = {};
    cost_results = [];
    
    CleanFiles = {}; %Store results without failed optimization results
    
    for fileIdx = 1:length(Files)
    
        load([directory,'/',Files(fileIdx).name]);
        
        %Clean the Result Files, Remove failed optimizations
        
        if (mod(sum(gait(:,1)),1) == 0) && (mod(sum(gait(:,2)),1) == 0) && sum(gait(:,1))<NumPhases && sum(gait(:,2))<NumPhases
            
            CleanFiles = {CleanFiles{:},Files(fileIdx)};
            
            gait_results = {gait_results{:},gait};
            
            cost_results = [cost_results,result_cost];
        
        end
        
    end
    
    disp(['Stride Period: ', num2str(Tend), 's'])
    
    disp('===========================================================')
    
    if isempty(CleanFiles)
        
        disp('All Optimization Trials Failed to Find Optimal Gait for This Speed......')
    
        disp('===========================================================')
        
    else
        
        [min_cost,min_cost_Idx] = min(cost_results);
        
        disp('OVERALL(GLOBAL OPTIMAL)')
        
        disp('-------------------------------------------------------')

        disp(['The File Name: ', CleanFiles{min_cost_Idx}.name])

        disp(['Optimal Gait: '])

        gait_results{min_cost_Idx}

        disp(['Corresponding Cost is: ', num2str(min_cost)])

        disp('===========================================================')

        disp('Loop Over All Results')

        disp('===========================================================')


        for fileIdx = 1:length(CleanFiles)

            disp('-------------------------------------------------------')

            disp(['Gait Optimizaiton Result for File ', num2str(fileIdx), ', FileName: ', CleanFiles{fileIdx}.name])

            disp('The Optimal Gait is:')

            gait_results{fileIdx}

            disp(['Optimal Cost: ', num2str(cost_results(fileIdx))])

            disp(['Difference to Minimum Cost (Current Cost - Minimum Cost): ', num2str(cost_results(fileIdx) - min_cost)])
            
            if cost_results(fileIdx) - min_cost <= 1
                
                disp('Cost Value is Equivalent to the Minimum Cost? -> YES')
                
            else
                
                disp('Cost Value is Equivalent to the Minimum Cost? -> No')
            
            end

            disp('-------------------------------------------------------')

        end
        
    end
    
end

diary off