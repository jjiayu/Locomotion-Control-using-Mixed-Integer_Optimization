%Enumerate all the gait for a Quadruped (8 Phases)

clear;
clc

if exist('GaitLibrary','dir') == 0
    mkdir('GaitLibrary');
end

GaitSavingNum = 1;

parfor i = 0:255
    %Pre-allocate Saving Locations
    GaitMatrixResult = zeros((2^4)^6,32);
    
    GaitSavingNum = 1; 
    for GaitCode = 1:(2^4)^6
        GaitDecCode = i*(2^4)^6 + GaitCode - 1;
        
        %Convert into Binary
        GaitBinCode = dec2bin(GaitDecCode,32) - '0';
        %Make Gait Matrix Format
        Gait = reshape(GaitBinCode, 4, 8);
    
        %Check if the gait is a periodical one
        FlagCnt = 0; %Count how many limbs meet the periodical requirement
    
        for limbNum = 1:4
            GaitLimb = Gait(limbNum,:); %Gait for a limb
            ContactChangeTimes = sum(abs(diff(GaitLimb)));
            if ContactChangeTimes >=1 && ContactChangeTimes <= 2
                FlagCnt = FlagCnt + 1;
            end
        end
    
        if FlagCnt == 4 %Meet Periodical Gait Criteria, save
            GaitMatrixResult(GaitSavingNum,:) = GaitBinCode;
            GaitSavingNum = GaitSavingNum + 1; %Counter + 1
        end
    end
    
    GaitMatrixResult = GaitMatrixResult(1:GaitSavingNum-1,:);
    parsave(i,GaitMatrixResult);
    
end

function parsave(process_number, gait_matrix_result)
save(['GaitLibrary/Slice_',num2str(process_number)], 'gait_matrix_result');
end

% 
% 
% parfor GaitDecCode = 0:(2^4)^8-1 %GaitDecCode: Gait Number Encoded using Decimals
%     
%     if mod(GaitDecCode,100000) == 0
%         disp([num2str(GaitDecCode/(2^4)^8*100),'%']);
%     end
%     
%     if mod(GaitDecCode,10000000) == 0
%         clc
%     end
%     
%     %Convert into Binary
%     GaitBinCode = dec2bin(GaitDecCode,32) - '0';
%     %Make Gait Matrix Format
%     Gait = reshape(GaitBinCode, 4, 8);
%     
%     %Check if the gait is a periodical one
%     FlagCnt = 0; %Count how many limbs meet the periodical requirement
%     
%     for limbNum = 1:4
%         GaitLimb = Gait(limbNum,:); %Gait for a limb
%         ContactChangeTimes = sum(abs(diff(GaitLimb)));
%         if ContactChangeTimes >=1 && ContactChangeTimes <= 2
%             FlagCnt = FlagCnt + 1;
%         end
%     end
%     
%     if FlagCnt == 4 %Meet Periodical Gait Criteria, save
%         save(['GaitLibrary/PeriodicalGait_',num2str(GaitSavingNum),'.mat'],'Gait');
%         GaitSavingNum = GaitSavingNum + 1; %Counter + 1
%     end
%     
% end