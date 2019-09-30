clear;
clc;

directory = uigetdir([pwd], 'Select Working Directory');

warning('off','all');

diary off
ResultCleanLog = strcat('Optimization-Results_Summary_with_Fixed_Gait', datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
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
        
        if strcmp(return_status.return_status,'KTR_RC_OPTIMAL_OR_SATISFACTORY')||...
           strcmp(return_status.return_status,'KTR_RC_NEAR_OPT')||...
           strcmp(return_status.return_status,'KTR_RC_FEAS_XTOL')||...
           strcmp(return_status.return_status,'KTR_RC_FEAS_NO_IMPROVE')||...
           strcmp(return_status.return_status,'KTR_RC_FEAS_FTOL')||...
           strcmp(return_status.return_status,'KTR_RC_ITER_LIMIT_FEAS')||...
           strcmp(return_status.return_status,'KTR_RC_TIME_LIMIT_FEAS')||...
           strcmp(return_status.return_status,'KTR_RC_FEVAL_LIMIT_FEAS')
            
            CleanFiles = {CleanFiles{:},Files(fileIdx)};
            
            gait_results = {gait_results{:},gait};
            
            cost_results = [cost_results,result_cost];
        
        end
        
    end
    
    disp(['Stride Period: ', num2str(Tend), 's'])
    
    disp('===========================================================')
    
    result_summary.speedList(SpeedIdx) = speed;
    
    if isempty(CleanFiles)
        
        disp('All Optimization Trials Failed to Find Optimal Gait for This Speed......')
    
        disp('===========================================================')
        
        %save results
        
        result_summary.costList(SpeedIdx) = inf;
        result_summary.gaitList{SpeedIdx} = inf;
        result_summary.opt_file_list{SpeedIdx} = inf;

    else
        
        [min_cost,min_cost_Idx] = min(cost_results);
        
        disp('OVERALL(GLOBAL OPTIMAL)')
        
        disp('-------------------------------------------------------')

        disp(['The File Name: ', CleanFiles{min_cost_Idx}.name])

        disp(['Optimal Gait: '])

        gait_results{min_cost_Idx}

        disp(['Corresponding Cost is: ', num2str(min_cost)])
        
        %Save results
        result_summary.costList(SpeedIdx) = min_cost;
        result_summary.gaitList{SpeedIdx} = gait_results{min_cost_Idx};
        result_summary.opt_file_list{SpeedIdx} = CleanFiles{min_cost_Idx}.name;
        
        disp('===========================================================')

        disp('Loop Over All Results')

        disp('===========================================================')


        for fileIdx = 1:length(CleanFiles)

            disp('-------------------------------------------------------')

            disp(['Optimizaiton Result for File ', num2str(fileIdx), ', FileName: ', CleanFiles{fileIdx}.name])

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

save([directory,'/','result_summary']);

diary off