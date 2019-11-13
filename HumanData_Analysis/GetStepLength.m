function [LeftStepLength,RightStepLength,StrangeGaitFlag] = GetStepLength(filepath)
%GETSTEPLENGTH Summary of this function goes here
%   Detailed explanation goes here
    
    %Default not special gait pattern: 0 , 1, 1, 0  
    StrangeGaitFlag = 0;
    
    %Load file
    Expresult = load(filepath);

    PFx_result_temp = Expresult.PFx_result;
    PHx_result_temp = Expresult.PHx_result;
    
    % return the index where the the element being minused is larger than
    % some threshold, so the starting index is the starting knot of the
    % swing phase, end index + 1 is termination knot (landing knot of the swing phase)
    FrontLeg_SwingKnot_Idxs = find(abs(diff(PFx_result_temp))>=0.001);
    HindLeg_SwingKnot_Idxs = find(abs(diff(PHx_result_temp))>=0.001);

    %Front Leg/Left Leg
    if sum(diff(FrontLeg_SwingKnot_Idxs)) == length(FrontLeg_SwingKnot_Idxs) - 1
        %FrontLeg_SwingStart_Idx = max(FrontLeg_SwingKnot_Idxs(1) - 1,1);
        FrontLeg_SwingStart_Idx = FrontLeg_SwingKnot_Idxs(1);
        FrontLeg_SwingEnd_Idx   = min(FrontLeg_SwingKnot_Idxs(end) + 1,length(PFx_result_temp));
        LeftStepLength = PFx_result_temp(FrontLeg_SwingEnd_Idx) - PFx_result_temp(FrontLeg_SwingStart_Idx);
    else %A special case: 0 , 1, 1, 0
        
        %Expresult.gait
        
        StrangeGaitFlag = 1;
        
        InterruptionPoints = find(diff(FrontLeg_SwingKnot_Idxs) > 1);
        if length(InterruptionPoints) == 1
            %disp('Special Case')
            %Index of the interruption index on the swingknot index list
            InterruptionIndex = find(diff(FrontLeg_SwingKnot_Idxs) > 1);
            Segment1_Start_Idx = 1;
            Segment1_End_Idx = FrontLeg_SwingKnot_Idxs(InterruptionIndex) + 1;
            Segment1_Dist = PFx_result_temp(Segment1_End_Idx) - PFx_result_temp(Segment1_Start_Idx);
            
            Segment2_Start_Idx = FrontLeg_SwingKnot_Idxs(InterruptionIndex+1);
            Segment2_End_Idx = min(FrontLeg_SwingKnot_Idxs(end) + 1,length(PFx_result_temp));
            Segment2_Dist = PFx_result_temp(Segment2_End_Idx) - PFx_result_temp(Segment2_Start_Idx);
            
            LeftStepLength = Segment1_Dist + Segment2_Dist;
        else
            LeftStepLength = inf;
        end
    end
    
    %Hind Leg/Right Leg
    if sum(diff(HindLeg_SwingKnot_Idxs)) == length(HindLeg_SwingKnot_Idxs) - 1
        %HindLeg_SwingStart_Idx = max(HindLeg_SwingKnot_Idxs(1) - 1,1);
        HindLeg_SwingStart_Idx = HindLeg_SwingKnot_Idxs(1);
        HindLeg_SwingEnd_Idx   = min(HindLeg_SwingKnot_Idxs(end) + 1,length(PHx_result_temp));
        RightStepLength = PHx_result_temp(HindLeg_SwingEnd_Idx) - PHx_result_temp(HindLeg_SwingStart_Idx);
    else
        %A special case: 0 , 1, 1, 0
        
        %Expresult.gait
        
        StrangeGaitFlag = 1;
        
        InterruptionPoints = find(diff(HindLeg_SwingKnot_Idxs) > 1);
        if length(InterruptionPoints) == 1
        %disp('Special Case')
        %Index of the interruption index on the swingknot index list
        InterruptionIndex = find(diff(HindLeg_SwingKnot_Idxs) > 1);
        Segment1_Start_Idx = 1;
        Segment1_End_Idx = HindLeg_SwingKnot_Idxs(InterruptionIndex)+1;
        Segment1_Dist = PHx_result_temp(Segment1_End_Idx) - PHx_result_temp(Segment1_Start_Idx);

        Segment2_Start_Idx = HindLeg_SwingKnot_Idxs(InterruptionIndex+1);
        Segment2_End_Idx = min(HindLeg_SwingKnot_Idxs(end) + 1,length(PFx_result_temp));
        Segment2_Dist = PHx_result_temp(Segment2_End_Idx) - PHx_result_temp(Segment2_Start_Idx);

        RightStepLength = Segment1_Dist + Segment2_Dist;
        else
        RightStepLength = inf;
        end

    end
end
