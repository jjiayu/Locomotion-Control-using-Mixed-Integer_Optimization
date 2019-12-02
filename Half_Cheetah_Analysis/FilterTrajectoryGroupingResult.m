% Filtering Trajectory Grouping Result with:
%   (1) A specific Gait Group Label (i.e. 'Galloping_Group_A')
%   (2) Each Scenario (StridePeriod and Speed Pair) will only
%   contain one instance

clear;
clc;

% Input Trajectory Grouping Result -> Table form
disp('Load Trajectory Grouping Result File (Table Form i.e. TabelForm_TrajectorywithGroupLabels20191202T175642.mat)')
[tragroup_file_name,tragroup_file_path] = uigetfile();
full_tragroup_file_path = [tragroup_file_path,tragroup_file_name];
disp(['Selected File: ',full_tragroup_file_path]);
load(full_tragroup_file_path)

% Specify the Desired Group Label
Desired_Group_Label = input('Decide the Desired Group Label (i.e. Galloping_Group_A, with ''): \n');

% Find Trajectories with specific groups
GroupedTrajectoriesCollectionTable.GaitGroupName = categorical(GroupedTrajectoriesCollectionTable.GaitGroupName);
Idx_of_Trajectories_with_SpecificName = GroupedTrajectoriesCollectionTable.GaitGroupName == Desired_Group_Label;

%Collect/show trajectories with selected group label
Tra_with_Specific_label = GroupedTrajectoriesCollectionTable(Idx_of_Trajectories_with_SpecificName,:);

showTras = input(['Decide if show the trajectories with label -> ',Desired_Group_Label,' : 1-> Yes 2-> No \n']);
if showTras == 1
    Tra_with_Specific_label
end

saveTras = input(['Decide if SAVE the trajectories with label -> ',Desired_Group_Label,' : 1-> Yes 2-> No \n']);
if showTras == 1
    %save files and write tables
    save([tragroup_file_path,'/',Desired_Group_Label,'.mat'],'Tra_with_Specific_label');
    writetable(Tra_with_Specific_label,[tragroup_file_path,'/',Desired_Group_Label,'.xls']);
end


% Make Table and Files with single entry for each speed and stride period
% pair

[~,UniqueEntryIdx] = unique(Tra_with_Specific_label(:,1:2),'rows');

Tra_with_Specific_label_SingleEntry = Tra_with_Specific_label(UniqueEntryIdx,:);

showTras = input(['Decide if show the trajectories with label -> ',Desired_Group_Label,' With Single Entry: 1-> Yes 2-> No \n']);
if showTras == 1
    Tra_with_Specific_label_SingleEntry
end

saveTras = input(['Decide if SAVE the trajectories with label -> ',Desired_Group_Label,' With Single Entry: 1-> Yes 2-> No \n']);
if showTras == 1
    %save files and write tables
    save([tragroup_file_path,'/',Desired_Group_Label,'_MeanInitCollection.mat'],'Tra_with_Specific_label_SingleEntry');
    writetable(Tra_with_Specific_label,[tragroup_file_path,'/',Desired_Group_Label,'_MeanInitCollection.xls']);
end

