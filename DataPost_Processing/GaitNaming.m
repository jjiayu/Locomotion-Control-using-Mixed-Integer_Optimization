function GaitName = GaitNaming(gait)
%GAITNAMING Summary of this function goes here
%   Detailed explanation goes here

%Default gait name
GaitName = "";

%duplicate the gait matrix, only take integer variables
gaitMatrix_doublephase = [gait(:,1:2);gait(:,1:2)];


if (mod(sum(gait(:,1)),1) ~= 0) && (mod(sum(gait(:,2)),1) ~= 0)
    GaitName = "N/A"; %No feasible results
else
    %Check Pronking gait first
    double_fly_support_flag = zeros(size(gaitMatrix_doublephase,1),1);
    for loop_Idx = 1:size(gaitMatrix_doublephase,1)
        if isequal(gaitMatrix_doublephase(loop_Idx,1),gaitMatrix_doublephase(loop_Idx,2))
            double_fly_support_flag(loop_Idx) = 1;
        else
            double_fly_support_flag(loop_Idx) = 0;
        end
    end
    if sum(double_fly_support_flag) == size(gaitMatrix_doublephase,1)
        GaitName = "Pronking";
    else %other gaits
        %find hind support phase
        HindContactIdx = [];
        for loop_Idx = 1:size(gaitMatrix_doublephase,1)
            if isequal(gaitMatrix_doublephase(loop_Idx,:),[0,1])
                HindContactIdx = [HindContactIdx,loop_Idx];
            end
        end

        if size(HindContactIdx) == 0 %no gait start from hind contact
            GaitName = "Unknown";
        else
            %Clean gait from hind support
            Gait_from_HindSupport = gaitMatrix_doublephase(HindContactIdx(1):HindContactIdx(1)+3,:);
            %remove phase with same contact config;
            clean_Gait_from_HindSupport = Gait_from_HindSupport;
            if isequal(clean_Gait_from_HindSupport(1,:),clean_Gait_from_HindSupport(end,:))
                clean_Gait_from_HindSupport(end,:) = [];
            end
            
            vanishing_phase_idx = [];
            for loop_Idx = 1:size(clean_Gait_from_HindSupport,1)-1
                if isequal(clean_Gait_from_HindSupport(loop_Idx,:),clean_Gait_from_HindSupport(loop_Idx+1,:))
                    vanishing_phase_idx = [vanishing_phase_idx,loop_Idx+1];
                end
            end                
            %remove redudant phases
            for loop_Idx = 1:size(vanishing_phase_idx)
                clean_Gait_from_HindSupport(vanishing_phase_idx(loop_Idx),:) = [];
            end
            
            %Assign Names
            if size(clean_Gait_from_HindSupport,1) == 2
                if isequal(clean_Gait_from_HindSupport(2,:),[1,0])
                    GaitName = "Trotting";
                else
                    GaitName = "Unknown";
                end
            elseif size(clean_Gait_from_HindSupport,1) == 3
                if isequal(clean_Gait_from_HindSupport(2,:),[1,1]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0])
                    GaitName = "Walking-S_H-D-F"; %Hind -> Double -> Front, commonly found in Half_cheetah model
                elseif isequal(clean_Gait_from_HindSupport(2,:),[1,0]) && isequal(clean_Gait_from_HindSupport(3,:),[1,1])
                    GaitName = "Walking-S_H-F-D"; %Hind -> Front -Double, commonly Found in Humanoid
                elseif isequal(clean_Gait_from_HindSupport(2,:),[0,0]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0])
                    GaitName = "Bounding-S_H-FLY-F";
                elseif isequal(clean_Gait_from_HindSupport(2,:),[1,0]) && isequal(clean_Gait_from_HindSupport(3,:),[0,0])
                    GaitName = "Bounding-S_H-F-FLY";
                else
                    GaitName = "Unknown";
                end
            elseif size(clean_Gait_from_HindSupport,1) == 4
                if isequal(clean_Gait_from_HindSupport(2,:),[1,1]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0]) && isequal(clean_Gait_from_HindSupport(4,:),[0,0])
                    GaitName = "Galloping_H-D-F-Fly"; %Gallop commonly found in half cheetah, Hind -> Double -> Front -> Fly
                elseif isequal(clean_Gait_from_HindSupport(2,:),[0,0]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0]) && isequal(clean_Gait_from_HindSupport(4,:),[1,1])
                    GaitName = "Galloping_H-Fly-F-D"; %Gallop commonly found in humanoid, Hind -> Fly -> Front -> Double
                elseif isequal(clean_Gait_from_HindSupport(2,:),[0,0]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0]) && isequal(clean_Gait_from_HindSupport(4,:),[0,0])
                    GaitName = "Bounding-D";
                elseif isequal(clean_Gait_from_HindSupport(2,:),[1,1]) && isequal(clean_Gait_from_HindSupport(3,:),[1,0]) && isequal(clean_Gait_from_HindSupport(4,:),[1,1])
                    GaitName = "Walking-D";
                else
                    GaitName = "Unknown";
                end
            else
                GaitName = "Unknown";
            end
        end
    end
end


end

