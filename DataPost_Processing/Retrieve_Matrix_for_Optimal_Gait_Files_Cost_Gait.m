clear;
clc;

directory = uigetdir(['~/Dropbox/'], 'Select Working Directory');

diary off
warning('off')

min_StridePeriod = input('Input Minimum Stride Period: \n');
max_StridePeriod = input('Input Maximum Stride Perdio: \n');
StridePeriod_Resolution = input('Input spacing for Stride Period Vector: \n');
StridePeriod_List = min_StridePeriod:StridePeriod_Resolution:max_StridePeriod;

disp(' ')

min_Speed = input('Input minimum Speed: \n');
max_Speed = input('Input maximum Speed: \n');
Speed_Resolution = input('Input spacing for Speed Vector: \n');
Speed_List = min_Speed:Speed_Resolution:max_Speed;

resultMatrix = cell(length(Speed_List),length(StridePeriod_List));

FileMatrix = strings(length(Speed_List),length(StridePeriod_List));
GaitMatrix = cell(length(Speed_List),length(StridePeriod_List));
GaitNameMatrix = strings(length(Speed_List),length(StridePeriod_List));
CostMatrix = zeros(length(Speed_List),length(StridePeriod_List));

for StridePeriod_Idx = 1:length(StridePeriod_List)
    StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriod_Idx)),'/']
%     if mod(StridePeriod_List(StridePeriod_Idx),1)==0
%         StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriod_Idx)),'.0/']
%     end
    
    for Speed_Idx = 1:length(Speed_List)  
        speed = Speed_List(Speed_Idx);
        
        disp('===========================================================')
        disp(['For Speed: ', num2str(speed), ' m/s'])
        disp(['Stride Period: ', num2str(StridePeriod_List(StridePeriod_Idx)), 's'])
        
        Files = dir(fullfile(StridePeriod_Path,['Speed-',num2str(speed),'-*.mat']));
        
        gait_results = {};
        cost_results = [];
        CleanFiles = {}; %Store results without failed optimization results
    
        for fileIdx = 1:length(Files)
    
            load([StridePeriod_Path,'/',Files(fileIdx).name]);
        
            %Clean the Result Files, Remove failed optimizations
            
            if exist('return_status','var') == 1
                if return_status.success == 1
                    CleanFiles = {CleanFiles{:},Files(fileIdx)};
                    gait_results = {gait_results{:},gait};
                    cost_results = [cost_results,result_cost];
                end
            else %no existence of resturn_status
                if (mod(sum(gait(:,1)),1) == 0) && (mod(sum(gait(:,2)),1) == 0) && (sum(gait(:,1))<NumPhases) && (sum(gait(:,2))<NumPhases)
                    CleanFiles = {CleanFiles{:},Files(fileIdx)};
                    gait_results = {gait_results{:},gait};
                    cost_results = [cost_results,result_cost];
                end
            end
        end
        
        if isempty(CleanFiles)
        
            disp('All Optimization Trials Failed to Find Optimal Gait for This Speed......')
            disp('===========================================================')
                
            %here put data processing algorithm to deal with failed exps
            
            result_collection.strideperiod_speed_pair = [StridePeriod_List(StridePeriod_Idx),Speed_List(Speed_Idx)];
            result_collection.OptimalGait = rand(4,3);
            result_collection.OptimalGaitName = "N/A";
            result_collection.OptimalFile = "N/A";
            result_collection.OptimalCost = inf;
            result_collection.SimilarLocalMinimaFileNames = "N/A";
            
            resultMatrix{end-Speed_Idx+1,StridePeriod_Idx} = result_collection;
            
            %FileMatrix(end-Speed_Idx+1,StridePeriod_Idx) = "N/A";
            %GaitMatrix{end-Speed_Idx+1,StridePeriod_Idx} = 0.5*ones(4,3);
            %GaitNameMatrix(end-Speed_Idx+1,StridePeriod_Idx) = "N/A";
            %CostMatrix(end-Speed_Idx+1,StridePeriod_Idx) = inf;
            
        else
            %here put data processing algorithms
            [min_cost,min_cost_Idx] = min(cost_results);
            disp('OVERALL(GLOBAL OPTIMAL)')
            disp('-------------------------------------------------------')
            disp(['The File Name: ', CleanFiles{min_cost_Idx}.name])
            disp(['Optimal Gait: '])
            gait_results{min_cost_Idx}
            disp(['Corresponding Cost is: ', num2str(min_cost)])
            disp('===========================================================')
            
            result_collection.strideperiod_speed_pair = [StridePeriod_List(StridePeriod_Idx),Speed_List(Speed_Idx)];
            result_collection.OptimalGait = gait_results{min_cost_Idx};
            result_collection.OptimalGaitName = GaitNaming(gait_results{min_cost_Idx});
            result_collection.OptimalFile = CleanFiles{min_cost_Idx}.name;
            result_collection.OptimalCost = min_cost;
            
            %Collect Local Minima with similar cost
            result_collection.SimilarLocalMinimaFileNames = {};
            for fileIdx = 1:length(CleanFiles)
                disp('-------------------------------------------------------')
                disp(['Gait Optimizaiton Result for File ', num2str(fileIdx), ', FileName: ', CleanFiles{fileIdx}.name])
                disp('The Optimal Gait is:')
                gait_results{fileIdx}
                disp(['Optimal Cost: ', num2str(cost_results(fileIdx))])
                disp(['Difference to Minimum Cost (Current Cost - Minimum Cost): ', num2str(cost_results(fileIdx) - min_cost),' In Percentage: ',num2str((cost_results(fileIdx) - min_cost)/min_cost*100),'%'])
                if ((cost_results(fileIdx) - min_cost)/min_cost*100 <= 0.1) && (fileIdx ~= min_cost_Idx)
                    disp('Cost Value is Equivalent to the Minimum Cost? -> YES')
                    result_collection.SimilarLocalMinimaFileNames = {result_collection.SimilarLocalMinimaFileNames{:},CleanFiles{fileIdx}.name};
                else
                    disp('Cost Value is Equivalent to the Minimum Cost? -> No')
                end
            disp('-------------------------------------------------------')
            end

            resultMatrix{end-Speed_Idx+1,StridePeriod_Idx} = result_collection;
            
        end
        
        %flip vector
        %FileList_currentStridePeriod = flipud(FileList_currentStridePeriod');
        %FileMatrix{:,StridePeriod_Idx} = FileList_currentStridePeriod;
    end
end


%Generate Gait Matrix
GaitNameMatrix = cell(size(resultMatrix));

for StridePeriodLoop_Idx = 1:size(resultMatrix,2)
    for SpeedLoop_Idx = 1:size(resultMatrix,1)
        GaitNameMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx} = GaitNaming(resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.OptimalGait);
    end
end

GaitNameMatrix_withAllLocalMinima = cell(size(resultMatrix));

%Generate Gait Matrix with similar Local Minima
for StridePeriodLoop_Idx = 1:size(resultMatrix,2)
    
    StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriodLoop_Idx)),'/']
    
%     if mod(StridePeriod_List(StridePeriod_Idx),1)==0
%         StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriodLoop_Idx)),'.0/']
%     else
%         StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriodLoop_Idx)),'/']
%     end
    
    for SpeedLoop_Idx = 1:size(resultMatrix,1)
        
        FileswithSimilarCost = resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.SimilarLocalMinimaFileNames;
        
        if isempty(FileswithSimilarCost) || strcmp(strcat(FileswithSimilarCost{:}),"N/A")
            GaitNameswithSimilarCost = [];
            resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.GaitNameswithSimilarCost = GaitNameswithSimilarCost;
        else
            GaitNameswithSimilarCost = [];
            for FileLoop_Idx = 1:length(FileswithSimilarCost)
                load([StridePeriod_Path,'/',FileswithSimilarCost{FileLoop_Idx}]);
                tempGaitName = GaitNaming(gait);
                GaitNameswithSimilarCost = [GaitNameswithSimilarCost,tempGaitName];
            end
            resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.GaitNameswithSimilarCost = GaitNameswithSimilarCost;  
        end

        resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.allLocalOptimalGaitWithSimilarCost = unique([resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.OptimalGaitName,GaitNameswithSimilarCost]);
        
        AllLocatMinimaGaitName = [];
        for Idx_temp = 1:length(resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.allLocalOptimalGaitWithSimilarCost)
            AllLocatMinimaGaitName = [AllLocatMinimaGaitName,resultMatrix{SpeedLoop_Idx,StridePeriodLoop_Idx}.allLocalOptimalGaitWithSimilarCost{Idx_temp},'/'];
        end
        
        GaitNameMatrix_withAllLocalMinima{SpeedLoop_Idx,StridePeriodLoop_Idx} = AllLocatMinimaGaitName;
%         GaitNameswithSimilarCost = [];
%         for FileLoop_Idx = 1:length(FileswithSimilarCost)
%             load([StridePeriod_Path,'/',Files(FileLoop_Idx).name]);
%             tempGaitName = GaitNaming(gait);
%             GaitNameswithSimilarCost = [GaitNameswithSimilarCost,tempGaitName];
%         end
    end
end




