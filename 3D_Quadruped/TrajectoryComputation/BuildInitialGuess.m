% Build Initial Guess 

%reset random seed genreator
rng shuffle

if gait_discovery_switch == 1 %Free Gait Discovery using MINLP
    DecisionVarsInit = lb_ContinuousDecisionVars + (ub_ContinuousDecisionVars - lb_ContinuousDecisionVars).*rand(1,length(lb_ContinuousDecisionVars));
    DecisionVarsInit = [DecisionVarsInit, randi([0,1], 1, VarLengthList(VarCategoryList == 'Clf') + VarLengthList(VarCategoryList == 'Clh') + VarLengthList(VarCategoryList == 'Crf') + VarLengthList(VarCategoryList == 'Crh'))];
elseif gait_discovery_switch == 2 %Fixed Gait Optimization
    DecisionVarsInit = lb_DecisionVars + (ub_DecisionVars - lb_DecisionVars).*rand(1,length(DecisionVars));
end

%Deal with Initial Guess of Switching times
%   Random Numbers sort in a specific order
Ts_init = lb_ContinuousDecisionVars(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end))) + ...
          (ub_ContinuousDecisionVars(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end))) - lb_ContinuousDecisionVars(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end)))).*rand(1,length(Ts_label));
Ts_init = sort(Ts_init,'ascend');
DecisionVarsInit(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end))) = Ts_init;

%DecisionVarsInit has to be column vector
DecisionVarsInit = DecisionVarsInit';
