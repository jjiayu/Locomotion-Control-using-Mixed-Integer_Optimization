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

FileMatrix = strings(length(Speed_List),length(StridePeriod_List));
GaitMatrix = cell(length(Speed_List),length(StridePeriod_List));
GaitNameMatrix = strings(length(Speed_List),length(StridePeriod_List));
CostMatrix = zeros(length(Speed_List),length(StridePeriod_List));

for StridePeriod_Idx = 1:length(StridePeriod_List)
    StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriod_Idx)),'/']
    if mod(StridePeriod_List(StridePeriod_Idx),1)==0
        StridePeriod_Path = [directory,'/4Phases_StridePeriod_',num2str(StridePeriod_List(StridePeriod_Idx)),'.0/']
    end
    
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
        
            if (mod(sum(gait(:,1)),1) == 0) && (mod(sum(gait(:,2)),1) == 0) && sum(gait(:,1))<NumPhases && sum(gait(:,2))<NumPhases
                CleanFiles = {CleanFiles{:},Files(fileIdx)};
                gait_results = {gait_results{:},gait};
                cost_results = [cost_results,result_cost];
            end
        
        end
        
        if isempty(CleanFiles)
        
            disp('All Optimization Trials Failed to Find Optimal Gait for This Speed......')
            disp('===========================================================')
                
            %here put data processing algorithm to deal with failed exps
            
            FileMatrix(end-Speed_Idx+1,StridePeriod_Idx) = "N/A";
            GaitMatrix{end-Speed_Idx+1,StridePeriod_Idx} = 0.5*ones(4,3);
            GaitNameMatrix(end-Speed_Idx+1,StridePeriod_Idx) = "N/A";
            CostMatrix(end-Speed_Idx+1,StridePeriod_Idx) = inf;
            
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
            
            FileMatrix(end-Speed_Idx+1,StridePeriod_Idx) = CleanFiles{min_cost_Idx}.name;
            GaitMatrix{end-Speed_Idx+1,StridePeriod_Idx} = gait_results{min_cost_Idx};
            CostMatrix(end-Speed_Idx+1,StridePeriod_Idx) = min_cost;
            
%             %Deal with GaitName Generation
%             %   Dublicate the gait
%             GaitTemp = [Gait;Gait];
%             %   
%             HindContactIdx = [];
%             for loop_idx = 1:size(GaitTemp,1)
%                 if GaitTemp(loop_idx,1:2) == [0,1] %Find non-pronking gait, all start from [0,1]
%                     HindContactIdx = [HindContactIdx,loop_idx];
%                 end
%             end
%             StartingIdx = min(HindContactIdx);
%             %   Get gait start from hind contact
%             GaitfromHindContact = GaitTemp(StartingIdx:StartingIdx+3,:);
%             %Clean Gait
%             if GaitfromHindContact(1,1:2) == GaitfromHindContact(end,1:2)
%                 GaitfromHindContact(1,3) = GaitfromHindContact(1,3) + GaitfromHindContact(end,3);
%                 GaitfromHindContact(end,:) = [];
%             else
%                 for loop_idx = 1:size(GaitfromHindContact,1)-1
%                     if GaitfromHindContact(loop_idx,1:2) == GaitfromHindContact(loop_idx+1,1:2)
%                         GaitfromHindContact(loop_idx,3) = GaitfromHindContact(loop_idx,3) + GaitfromHindContact(loop_idx+1,2);
%                         GaitfromHindContact(loop_idx+1,:) = [];
%                     end
%                 end
%             end
%             
%             %judge Gait
%             if GaitfromHindContact == [0,1;0,0;1,0;1,1] %Galloping
%                 GaitNameMatrix(end-Speed_Idx+1,StridePeriod_Idx) = "Galloping"
%             end
            
        end
        
        %flip vector
        %FileList_currentStridePeriod = flipud(FileList_currentStridePeriod');
        %FileMatrix{:,StridePeriod_Idx} = FileList_currentStridePeriod;
    end
end


