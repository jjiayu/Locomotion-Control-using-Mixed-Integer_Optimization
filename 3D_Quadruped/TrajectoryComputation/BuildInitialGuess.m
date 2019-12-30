% Build Initial Guess 



if gait_discovery_switch == 1 %Free Gait Discovery using MINLP
    DecisionVarsInit = lb_ContinuousDecisionVars + (ub_ContinuousDecisionVars - lb_ContinuousDecisionVars).*rand(1,length(lb_ContinuousDecisionVars));
    DecisionVarsInit = [DecisionVarsInit, randi([0,1], 1, VarLengthList(VarCategoryList == 'Clf') + VarLengthList(VarCategoryList == 'Clh') + VarLengthList(VarCategoryList == 'Crf') + VarLengthList(VarCategoryList == 'Crh'))];
elseif gait_discovery_switch == 2 %Fixed Gait Optimization
    DecisionVarsInit = lb_DecisionVars + (ub_DecisionVars - lb_DecisionVars).*rand(1,length(DecisionVars));
end

%DecisionVarsInit has to be column vector
DecisionVarsInit = DecisionVarsInit';
