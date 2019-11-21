function [GaitName, GroupName] = TrajectoryGroupClassifier(DiscoveredGaitName, DiscoveredGaitPattern,PrintGroups)
%Identify what is the gait, what group the gait should go

    %--------------------------------------------------------------------------
    % DataBase for Different Group of Gaits
    %   For Galloping
    GallopingTemplate = [1,0;...
                         0,0;...
                         0,1;...
                         1,1];

    GallopingTemplateDouble = [GallopingTemplate;GallopingTemplate];

    Galloping_Group_A = GallopingTemplateDouble(1:4,:);
    Galloping_Group_B = GallopingTemplateDouble(2:5,:);
    Galloping_Group_C = GallopingTemplateDouble(3:6,:);
    Galloping_Group_D = GallopingTemplateDouble(4:7,:);

    %   For Walking-D
    Walking_D_Template = [1,0;...
                          1,1;...
                          0,1;...
                          1,1];

    Walking_D_TemplateDouble = [Walking_D_Template;Walking_D_Template];

    Walking_D_Group_A = Walking_D_TemplateDouble(1:4,:);
    Walking_D_Group_B = Walking_D_TemplateDouble(2:5,:);
    Walking_D_Group_C = Walking_D_TemplateDouble(3:6,:);
    Walking_D_Group_D = Walking_D_TemplateDouble(4:7,:);

    %   For Bounding-D
    Bounding_D_Template = [0,0;...
                           0,1;...
                           0,0;...
                           1,0];

    Bounding_D_TemplateDouble = [Bounding_D_Template;Bounding_D_Template];

    Bounding_D_Group_A = Bounding_D_TemplateDouble(1:4,:);
    Bounding_D_Group_B = Bounding_D_TemplateDouble(2:5,:);
    Bounding_D_Group_C = Bounding_D_TemplateDouble(3:6,:);
    Bounding_D_Group_D = Bounding_D_TemplateDouble(4:7,:);

    %   For Walking-S
    Walking_S_3Phases_Template = [0,1;...
                                  1,1;...
                                  1,0];
    %       Group A -> duplicate phase 1                          
    Walking_S_Group_A_Template = [Walking_S_3Phases_Template(1,:);Walking_S_3Phases_Template];
    Walking_S_Group_A_TemplateDouble = [Walking_S_Group_A_Template;Walking_S_Group_A_Template];

    Walking_S_Group_A_1 = Walking_S_Group_A_TemplateDouble(1:4,:);
    Walking_S_Group_A_2 = Walking_S_Group_A_TemplateDouble(2:5,:);
    Walking_S_Group_A_3 = Walking_S_Group_A_TemplateDouble(3:6,:);
    Walking_S_Group_A_4 = Walking_S_Group_A_TemplateDouble(4:7,:);

    %       Group B -> duplicate phase 2      
    Walking_S_Group_B_Template = [Walking_S_3Phases_Template(1,:);Walking_S_3Phases_Template(2,:);Walking_S_3Phases_Template(2,:);Walking_S_3Phases_Template(3,:)];
    Walking_S_Group_B_TemplateDouble = [Walking_S_Group_B_Template;Walking_S_Group_B_Template];

    Walking_S_Group_B_1 = Walking_S_Group_B_TemplateDouble(1:4,:);
    Walking_S_Group_B_2 = Walking_S_Group_B_TemplateDouble(2:5,:);
    Walking_S_Group_B_3 = Walking_S_Group_B_TemplateDouble(3:6,:);
    Walking_S_Group_B_4 = Walking_S_Group_B_TemplateDouble(4:7,:);

    %       Group C -> duplicated phase 3
    Walking_S_Group_C_Template = [Walking_S_3Phases_Template(1:2,:);Walking_S_3Phases_Template(3,:);Walking_S_3Phases_Template(3,:)];
    Walking_S_Group_C_TemplateDouble = [Walking_S_Group_C_Template;Walking_S_Group_C_Template];

    Walking_S_Group_C_1 = Walking_S_Group_C_TemplateDouble(1:4,:);
    Walking_S_Group_C_2 = Walking_S_Group_C_TemplateDouble(2:5,:);
    Walking_S_Group_C_3 = Walking_S_Group_C_TemplateDouble(3:6,:);
    Walking_S_Group_C_4 = Walking_S_Group_C_TemplateDouble(4:7,:);

    %   For Bounding-S
    Bounding_S_3Phases_Template = [0,1;...
                                  1,0;...
                                  0,0];
    %       Group A -> duplicate phase 1                          
    Bounding_S_Group_A_Template = [Bounding_S_3Phases_Template(1,:);Bounding_S_3Phases_Template];
    Bounding_S_Group_A_TemplateDouble = [Bounding_S_Group_A_Template;Bounding_S_Group_A_Template];

    Bounding_S_Group_A_1 = Bounding_S_Group_A_TemplateDouble(1:4,:);
    Bounding_S_Group_A_2 = Bounding_S_Group_A_TemplateDouble(2:5,:);
    Bounding_S_Group_A_3 = Bounding_S_Group_A_TemplateDouble(3:6,:);
    Bounding_S_Group_A_4 = Bounding_S_Group_A_TemplateDouble(4:7,:);

    %       Group B -> duplicate phase 2      
    Bounding_S_Group_B_Template = [Bounding_S_3Phases_Template(1,:);Bounding_S_3Phases_Template(2,:);Bounding_S_3Phases_Template(2,:);Bounding_S_3Phases_Template(3,:)];
    Bounding_S_Group_B_TemplateDouble = [Bounding_S_Group_B_Template;Bounding_S_Group_B_Template];

    Bounding_S_Group_B_1 = Bounding_S_Group_B_TemplateDouble(1:4,:);
    Bounding_S_Group_B_2 = Bounding_S_Group_B_TemplateDouble(2:5,:);
    Bounding_S_Group_B_3 = Bounding_S_Group_B_TemplateDouble(3:6,:);
    Bounding_S_Group_B_4 = Bounding_S_Group_B_TemplateDouble(4:7,:);

    %       Group C -> duplicated phase 3
    Bounding_S_Group_C_Template = [Bounding_S_3Phases_Template(1:2,:);Bounding_S_3Phases_Template(3,:);Bounding_S_3Phases_Template(3,:)];
    Bounding_S_Group_C_TemplateDouble = [Bounding_S_Group_C_Template;Bounding_S_Group_C_Template];

    Bounding_S_Group_C_1 = Bounding_S_Group_C_TemplateDouble(1:4,:);
    Bounding_S_Group_C_2 = Bounding_S_Group_C_TemplateDouble(2:5,:);
    Bounding_S_Group_C_3 = Bounding_S_Group_C_TemplateDouble(3:6,:);
    Bounding_S_Group_C_4 = Bounding_S_Group_C_TemplateDouble(4:7,:);

    %--------------------------------------------------------------------------
    if PrintGroups == 1

        diary('Gait_Pattern_Groups.txt');

        disp('===============================================================')
        disp('Galloping')
        disp('===============================================================')
        disp('Galloping Group A:')
        disp('---------------------------------------------------------------')
        disp(Galloping_Group_A)
        disp('---------------------------------------------------------------')
        disp('Galloping Group B:')
        disp('---------------------------------------------------------------')
        disp(Galloping_Group_B)
        disp('---------------------------------------------------------------')
        disp('Galloping Group C:')
        disp('---------------------------------------------------------------')
        disp(Galloping_Group_C)
        disp('---------------------------------------------------------------')
        disp('Galloping Group D:')
        disp('---------------------------------------------------------------')
        disp(Galloping_Group_D)
        disp('---------------------------------------------------------------')
        disp(' ')

        disp('===============================================================')
        disp('Walking_D')
        disp('===============================================================')
        disp('Walking_D Group A:')
        disp('---------------------------------------------------------------')
        disp(Walking_D_Group_A)
        disp('---------------------------------------------------------------')
        disp('Walking_D Group B:')
        disp('---------------------------------------------------------------')
        disp(Walking_D_Group_B)
        disp('---------------------------------------------------------------')
        disp('Walking_D Group C:')
        disp('---------------------------------------------------------------')
        disp(Walking_D_Group_C)
        disp('---------------------------------------------------------------')
        disp('Walking_D Group D:')
        disp('---------------------------------------------------------------')
        disp(Walking_D_Group_D)
        disp('---------------------------------------------------------------')
        disp(' ')

        disp('===============================================================')
        disp('Bounding_D')
        disp('===============================================================')
        disp('Bounding_D Group A:')
        disp('---------------------------------------------------------------')
        disp(Bounding_D_Group_A)
        disp('---------------------------------------------------------------')
        disp('Bounding_D Group B:')
        disp('---------------------------------------------------------------')
        disp(Bounding_D_Group_B)
        disp('---------------------------------------------------------------')
        disp('Bounding_D Group C:')
        disp('---------------------------------------------------------------')
        disp(Bounding_D_Group_C)
        disp('---------------------------------------------------------------')
        disp('Bounding_D Group D:')
        disp('---------------------------------------------------------------')
        disp(Bounding_D_Group_D)
        disp('---------------------------------------------------------------')
        disp(' ')

        disp('===============================================================')
        disp('Walking_S')
        disp('===============================================================')
        disp('Walking_S Group A1:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_A_1)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group A2:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_A_2)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group A3:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_A_3)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group A4:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_A_4)
        disp('---------------------------------------------------------------')
        disp('---------------------------------------------------------------')
        disp('Walking_S Group B1:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_B_1)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group B2:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_B_2)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group B3:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_B_3)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group B4:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_B_4)
        disp('---------------------------------------------------------------')
        disp('---------------------------------------------------------------')
        disp('Walking_S Group C1:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_C_1)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group C2:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_C_2)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group C3:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_C_3)
        disp('---------------------------------------------------------------')
        disp('Walking_S Group C4:')
        disp('---------------------------------------------------------------')
        disp(Walking_S_Group_C_4)
        disp('---------------------------------------------------------------')
        disp(' ')

        disp('===============================================================')
        disp('Bounding_S')
        disp('===============================================================')
        disp('Bounding_S Group A1:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_A_1)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group A2:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_A_2)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group A3:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_A_3)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group A4:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_A_4)
        disp('---------------------------------------------------------------')
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group B1:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_B_1)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group B2:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_B_2)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group B3:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_B_3)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group B4:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_B_4)
        disp('---------------------------------------------------------------')
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group C1:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_C_1)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group C2:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_C_2)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group C3:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_C_3)
        disp('---------------------------------------------------------------')
        disp('Bounding_S Group C4:')
        disp('---------------------------------------------------------------')
        disp(Bounding_S_Group_C_4)
        disp('---------------------------------------------------------------')
        disp('---------------------------------------------------------------')

    end
    %--------------------------------------------------------------------------

    %--------------------------------------------------------------------------
    % Start Grouping
    % Unknown Scenarios as Default
    GaitName = 'N/A';
    GroupName = 'N/A';

    % Give Group names

    if contains(DiscoveredGaitName,'Walking-D') == 1
        
        GaitName = 'Walking-D';
        
        if isequal(DiscoveredGaitPattern(:,1:2),Walking_D_Group_A)
            GroupName = 'Walking_D_Group_A';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_D_Group_B)
            GroupName = 'Walking_D_Group_B';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_D_Group_C)
            GroupName = 'Walking_D_Group_C';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_D_Group_D)
            GroupName = 'Walking_D_Group_D';
        else
            GroupName = 'Unknown_Group_Name_for_Walking_D';
        end

    elseif contains(DiscoveredGaitName,'Walking-S') == 1
        
        GaitName = 'Walking-S';
        
        if isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_A_1)
            GroupName = 'Walking_S_Group_A1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_A_2)
            GroupName = 'Walking_S_Group_A2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_A_3)
            GroupName = 'Walking_S_Group_A3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_A_4)
            GroupName = 'Walking_S_Group_A4';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_B_1)
            GroupName = 'Walking_S_Group_B1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_B_2)
            GroupName = 'Walking_S_Group_B2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_B_3)
            GroupName = 'Walking_S_Group_B3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_B_4)
            GroupName = 'Walking_S_Group_B4';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_C_1)
            GroupName = 'Walking_S_Group_C1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_C_2)
            GroupName = 'Walking_S_Group_C2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_C_3)
            GroupName = 'Walking_S_Group_C3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Walking_S_Group_C_4)
            GroupName = 'Walking_S_Group_C4';
        else
            GroupName = 'Unknown_Group_Name_for_Walking_S';
        end

    %elseif contains(DiscoveredGaitName,'Trotting') == 1

    elseif contains(DiscoveredGaitName,'Gallop') == 1

        GaitName = 'Gallop';
        
        if isequal(DiscoveredGaitPattern(:,1:2),Galloping_Group_A)
            GroupName = 'Galloping_Group_A';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Galloping_Group_B)
            GroupName = 'Galloping_Group_B';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Galloping_Group_C)
            GroupName = 'Galloping_Group_C';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Galloping_Group_D)
            GroupName = 'Galloping_Group_D';
        else
            GroupName = 'Unknown_Group_Name_for_Galloping';
        end
        
    elseif contains(DiscoveredGaitName,'Bounding-S') == 1
        
        GaitName = 'Bounding-S';
        
        if isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_A_1)
            GroupName = 'Bounding_S_Group_A1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_A_2)
            GroupName = 'Bounding_S_Group_A2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_A_3)
            GroupName = 'Bounding_S_Group_A3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_A_4)
            GroupName = 'Bounding_S_Group_A4';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_B_1)
            GroupName = 'Bounding_S_Group_B1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_B_2)
            GroupName = 'Bounding_S_Group_B2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_B_3)
            GroupName = 'Bounding_S_Group_B3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_B_4)
            GroupName = 'Bounding_S_Group_B4';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_C_1)
            GroupName = 'Bounding_S_Group_C1';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_C_2)
            GroupName = 'Bounding_S_Group_C2';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_C_3)
            GroupName = 'Bounding_S_Group_C3';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_S_Group_C_4)
            GroupName = 'Bounding_S_Group_C4';
        else
            GroupName = 'Unknown_Group_Name_for_Bounding_S';
        end

    elseif contains(DiscoveredGaitName,'Bounding-D') == 1
        
        GaitName = 'Bounding-D';
        
        if isequal(DiscoveredGaitPattern(:,1:2),Bounding_D_Group_A)
            GroupName = 'Bounding_D_Group_A';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_D_Group_B)
            GroupName = 'Bounding_D_Group_B';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_D_Group_C)
            GroupName = 'Bounding_D_Group_C';
        elseif isequal(DiscoveredGaitPattern(:,1:2),Bounding_D_Group_D)
            GroupName = 'Bounding_D_Group_D';
        else
            GroupName = 'Unknown_Group_Name_for_Bounding_D';
        end
        
    else
        
        GaitName = 'Unknown';
        GroupName = 'Unknown Gait Group in Unknown Gait';

    %elseif contains(GaitNameTemp,'Pronking') == 1

    %elseif contains(GaitNameTemp,'Unknown') == 1

    end


end

