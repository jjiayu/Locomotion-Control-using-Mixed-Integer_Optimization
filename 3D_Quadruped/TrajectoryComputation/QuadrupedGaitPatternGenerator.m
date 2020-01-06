function [Clf,Clh,Crf,Crh, GaitName] = QuadrupedGaitPatternGenerator(GaitNumber)
%QUADRUPEDGAITPATTERNGENERATOR Summary of this function goes here
%   Detailed explanation goes here


% List of Gaits --> Synchronise with Parameter_Template.m, ParameterSetup.m
%   1  -> Lateral Sequence Walk from WikiPedia
%   2  -> Four-Beat Walking from Remy's Group
%   3  -> Walking Trot (Also Called Two-Beat Walking from Remy's Paper)
%   4  -> Running Trot (Trotting with a Flying Phase)
%   5  -> Tolting from Remy's Paper
%   6  -> Pacing
%   7  -> Amble
%   8  -> Canter
%   9  -> Transverse Gallop
%   10 -> Rotary Gallop

    %Note the Sequence we assign the gait sequece becomes *Clh*,*Clf*,Crf,Crh
    %But the output of the function becomese *Clf*,*Clh*,Crf,Crh, which is
    %consistent with respect to the optimization programe
    %Total Number of Phase is set to be 8 for consistency considerations
    
    if GaitNumber == 1 %Lateral Sequence Walk
        Clh = [1,1,1,1,1,1,0,0]';
        Clf = [1,1,1,0,0,1,1,1]';
        Crf = [0,1,1,1,1,1,1,0]';
        Crh = [1,1,0,0,1,1,1,1]';
        GaitName = "LateralSequenceWalk";
    elseif GaitNumber == 2 % Four-Beat Walking from Remy's Paper
        Clh = [1,1,1,1,0,0,0,1]';
        Clf = [0,1,1,1,1,1,0,0]';
        Crf = [1,1,0,0,0,1,1,1]';
        Crh = [0,0,0,1,1,1,1,1]';
        GaitName = "FourBeatWalking";
    elseif GaitNumber == 3 % Walking Trot (Also Called Two-Beat Walking in Remy's Paper)
        Clh = [1,1,1,1,1,1,0,0]';
        Clf = [1,1,0,0,1,1,1,1]';
        Crf = [1,1,1,1,1,1,0,0]';
        Crh = [1,1,0,0,1,1,1,1]';
        GaitName = "WalkingTrot";
    elseif GaitNumber == 4 % Running Trot (Trotting with a Flying Phase)
        Clh = [1,1,0,0,0,0,0,0]';
        Clf = [0,0,0,0,1,1,0,0]';
        Crf = [1,1,0,0,0,0,0,0]';
        Crh = [0,0,0,0,1,1,0,0]';
        GaitName = "RunningTrot";
    elseif GaitNumber == 5 % Tolting from Remy's Paper
        Clh = [1,1,0,0,0,0,0,1]';
        Clf = [0,1,1,1,0,0,0,0]';
        Crf = [0,0,0,0,0,1,1,1]';
        Crh = [0,0,0,1,1,1,0,0]';
        GaitName = "Tolting";
    elseif GaitNumber == 6 % Pacing
        Clh = [1,1,0,0,0,0,0,0]';
        Clf = [1,1,0,0,0,0,0,0]';
        Crf = [0,0,0,0,1,1,0,0]';
        Crh = [0,0,0,0,1,1,0,0]';
        GaitName = "Pacing";
    elseif GaitNumber == 7 % Amble
        Clh = [0,0,0,0,1,1,1,1]';
        Clf = [0,0,1,1,1,1,0,0]';
        Crf = [1,1,0,0,0,0,1,1]';
        Crh = [1,1,1,1,0,0,0,0]';
        GaitName = "Amble";
    elseif GaitNumber == 8 % Canter
        Clh = [0,0,0,0,0,0,1,1]';
        Clf = [0,0,0,0,1,1,0,0]';
        Crf = [0,0,0,0,0,0,1,1]';
        Crh = [1,1,0,0,0,0,0,0]';
        GaitName = "Canter";
    elseif GaitNumber == 9 % Transverse Gallop
        Clh = [1,1,0,0,0,0,0,0]';
        Clf = [0,0,0,1,1,1,0,0]';
        Crf = [0,0,0,0,0,1,1,0]';
        Crh = [0,1,1,1,0,0,0,0]';
        GaitName = "TransverseGallop";
    elseif GaitNumber == 10 % Rotary Gallop
        Clh = [1,1,0,0,0,0,0,0]';
        Clf = [0,0,0,0,0,1,1,0]';
        Crf = [0,0,0,0,1,1,0,0]';
        Crh = [0,1,1,0,0,0,0,0]';
        GaitName = "Rotary Gallop";
    elseif GaitNumber == 11 % Bounding
        Clh = [0,0,0,0,1,1,0,0]';
        Clf = [1,1,0,0,0,0,0,0]';
        Crf = [1,1,0,0,0,0,0,0]';
        Crh = [0,0,0,0,1,1,0,0]';
        GaitName = "Bounding";
    end

    %Display Gait Pattern
    warning('NOTE 1: For display, Gait Matrix Sequence Becomes: LH -> LF -> RF -> RH')
    warning('NOTE 2: Each LINE Represnet the Contact Sequence of a Limb, *BUT* the Gait Pattern in Programm are Stored as Column Vectors')
    
    disp(' ')
    disp(GaitName)
   
    [Clh,...
     Clf,...
     Crf,...
     Crh]'

end

