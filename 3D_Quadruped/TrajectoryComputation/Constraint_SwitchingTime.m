function [g, lbg, ubg] = Constraint_SwitchingTime(Ts,Phaselb,Tend)
%CONSTRAINT_SWITCHINGTIME Switching Time Constraint

    g = {};
    lbg = [];
    ubg = [];
    
    Ts = [0;Ts];
    
    for i = 1:length(Ts)-1
        EqTemp = Ts(i+1) - Ts(i);
        g   = {g{:},EqTemp};
        lbg = [lbg; Phaselb];
        ubg = [ubg; Tend];
    end
    
end

