function [CF, CH, GaitName] = Gait_Selection(GaitNumber)
    
    CF = inf;
    CH = inf;
    GaitName = "Undefined";
    
    if GaitNumber == 1 %Walking-D (Symmetric Walking)
        CF = [1,1,0,1]';
        CH = [0,1,1,1]';
        [CF,CH]'
        GaitName = "Walking_D"
    elseif GaitNumber == 2 %Trotting
        CF = [1,1,0,0]';
        CH = [0,0,1,1]';
        [CF,CH]'
        GaitName = "Trotting"
    elseif GaitNumber == 3 %Galloping
        CF = [1,0,0,1]';
        CH = [0,0,1,1]';
        [CF,CH]'
        GaitName = "Galloping"
    elseif GaitNumber == 4 %Bounding-D (Symmetric Bounding)
        CF = [0,0,0,1]';
        CH = [0,1,0,0]';
        [CF,CH]'
        GaitName = "Bounding_D"
    elseif GaitNumber == 5 %Pronking
        CF = [0,1,1,0]';
        CH = [0,1,1,0]';
        [CF,CH]'
        GaitName = "Pronking"
    elseif GaitNumber == 6 %Walking-S (Asymmetric Walking/Galloping without flying phase) (3-Phases)
%         CF = [0,1,1,1]';
%         CH = [1,1,1,0]';
%         CF = [0,0,1,1]';
%         CH = [1,1,1,0]';
        CF = [0,1,1]';
        CH = [1,1,0]';
        [CF,CH]'
        GaitName = "Walking_S"
    elseif GaitNumber == 7 %Bounding-S (Asymmetric Bounding/Galloping without double support phase) (3Phases)
        CF = [0,1,0]';
        CH = [1,0,0]';
        [CF,CH]'
        GaitName = "Bounding_S"
    elseif GaitNumber == 8 %User Defined Gait
        CF = input('Specify Contact Sequence for the Front leg (a column vector): \n');
        CH = input('Specify Contact Sequence for the Hind leg (a column vector): \n');
        [CF,CH]'
        GaitName = "UserDefined"
    % elseif user_defined_gait == 6 %Walking-S-2DoubleSupportPhase (Asymmetric Walking/Galloping without flying phase)
    %     CF = [0,1,1,1]';
    %     CH = [1,1,1,0]';
    %     [CF,CH]'
    % elseif user_defined_gait == 7 %Walking-S-2HindSupportPhase
    %     CF = [0,0,1,1]';
    %     CH = [1,1,1,0]';
    %     %CF = [0,1,1]';
    %     %CH = [1,1,0]';
    %     %CF = [0,0,1,1,1,1]';
    %     %CH = [1,1,1,1,0,0]';
    %     [CF,CH]'
    % elseif user_defined_gait == 8 %Walking-S-2FrontSupportPhase
    %     CF = [0,1,1,1]';
    %     CH = [1,1,0,0]';
    %     [CF,CH]'
    % elseif user_defined_gait == 9 %Bounding-S (Asymmetric Bounding/Galloping without double support phase)
    %     CF = [0,1,0,0]';
    %     CH = [1,0,0,0]';
    %     %CF = [0,1,0]';
    %     %CH = [1,0,0]';
    %     [CF,CH]'
    % elseif user_defined_gait == 10 %Bounding-S-2HindSupportPhase
    %     CF = [0,0,1,0]';
    %     CH = [1,1,0,0]';
    %     [CF,CH]'
    % elseif user_defined_gait == 11 %Bounding-S-2FrontSupportPhase
    %     CF = [0,1,1,0]';
    %     CH = [1,0,0,0]';
    %     [CF,CH]'
    end
    % CF = input('Define the contact sequence for Front Leg (CF), a column vector: ');
    % CH = input('Define the contact sequence for Hind Leg (CH), a column vector: ');
    % CF = CF';
    % CH = CH';

end

