%ydot as the major axis


StridePeriod_Min = 0.4;
StridePeriod_Max = 1.6;
StridePeriod_res = 0.2;

StridePeriodList = StridePeriod_Min:StridePeriod_res:StridePeriod_Max;

Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','x',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','xdot',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','y',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','theta',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','thetadot',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFx',StridePeriodList,StridePeriodVector,SpeedVector)
Plot_StateSpace_Trajectories(MINLP_Result_DataBase,'ydot','FFy',StridePeriodList,StridePeriodVector,SpeedVector)


% 
% %Find ydot's max min value for axis limits
% ydot_array = [];
% for idx = 1:length(MINLP_Result_DataBase)
%     ydot_array = [ydot_array;MINLP_Result_DataBase{idx}.ydot_result];
% end
% ydot_max = max(ydot_array) + abs(0.1*max(ydot_array));
% ydot_min = min(ydot_array) - abs(0.1*min(ydot_array));
% 
% 
% % ydot vs y
% %Find y's max min value for axis limits
%  y_array = [];
%  for idx = 1:length(MINLP_Result_DataBase)
%     y_array = [y_array;MINLP_Result_DataBase{idx}.y_result];
%  end
% y_max = max(y_array) + abs(0.1*max(y_array));
% y_min = min(y_array) - abs(0.1*min(y_array));
% 
%  
% for StridePeriodIdx = 1:length(StridePeriodList)
% 
%     subplot(3,3,StridePeriodIdx)
%     hold on
%     for idx = 1:length(MINLP_Result_DataBase)
%         if StridePeriodVector(idx) >= StridePeriodList(StridePeriodIdx) - StridePeriodList(StridePeriodIdx)*0.1 && StridePeriodVector(idx) <= StridePeriodList(StridePeriodIdx) + StridePeriodList(StridePeriodIdx)*0.1
%             plot(MINLP_Result_DataBase{idx}.ydot_result,MINLP_Result_DataBase{idx}.y_result,'LineWidth',2,'DisplayName',['Speed-',num2str(SpeedVector(idx))]);
%         end
%     end
%     
%     xlim([ydot_min,ydot_max])
%     ylim([y_min,y_max])
%     xlabel('ydot')
%     ylabel('y')
%     title(['Stride Period: ', num2str(StridePeriodList(StridePeriodIdx))])
%     lgd(StridePeriodIdx) = legend();
%     
%     hold off
% end
% suptitle('State Space Trajectory, ydot v.s. y')
% 
% for StridePeriodIdx = 1:length(StridePeriodList)
%     lgd(StridePeriodIdx).Location = 'northeastoutside';
% end