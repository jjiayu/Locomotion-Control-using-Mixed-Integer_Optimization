database = {};
for outerloop_idx = 1:size(resultMatrix,1)
    for innerloop_idx = 1:size(resultMatrix,2)
        for GaitNameLoop_Idx = 1:length(resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost)
            GaitNameTemp = resultMatrix{outerloop_idx,innerloop_idx}.allLocalOptimalGaitWithSimilarCost(GaitNameLoop_Idx);
            
            if contains(GaitNameTemp,'Unknown') ~= 1
                if contains(GaitNameTemp,'Walking-D') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Walking-D",1};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Walking-S') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Walking-S",2};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Trotting') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Trotting",3};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Gallop') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Galloping",4};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Bounding-S') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Bounding-S",5};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Bounding-D') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Bounding-D",6};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Pronking') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Pronking",7};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Sync-FrontLand') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Sync-FrontLand",8};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'Sync-HindLand') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Sync-HindLand",9};
                    database(size(database,1)+1,:) = TempLine;
                elseif contains(GaitNameTemp,'N/A') == 1
                    TempLine = {resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(1),resultMatrix{outerloop_idx,innerloop_idx}.strideperiod_speed_pair(2),"Infeasible",10};
                    database(size(database,1)+1,:) = TempLine;
                end
            end
        end
    end
end

DatabaseTable = cell2table(database,'VariableNames',["StridePeriod","Speed","Gait","GaitNumber"]);
writetable(DatabaseTable,[directory,['/database-',datestr(datetime('now'), 30),'.csv']]);
