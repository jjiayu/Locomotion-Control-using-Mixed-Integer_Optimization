function [g,lbg,ubg] = Constraint_ForceActivation(Fx,Fz,C,k,PhaseIdx,theta_slope,Mf)
%CONSTRAINT_FORCEACTIVATION Activation and Deactivation of Forces Due to
%complementarity constraint

    g = {};
    lbg = [];
    ubg = [];
    
    %Set 1:
    Eqtemp = [cos(theta_slope), sin(theta_slope); -sin(theta_slope), cos(theta_slope)]*[Fx(k);Fz(k)] - Mf*C(PhaseIdx);
    g = {g{:}, Eqtemp(1), Eqtemp(2)};
    lbg = [lbg; -inf; -inf];
    ubg = [ubg;    0;    0];
    
    %Set 2:
    Eqtemp = [cos(theta_slope), sin(theta_slope); -sin(theta_slope), cos(theta_slope)]*[Fx(k);Fz(k)] + Mf*C(PhaseIdx);
    g = {g{:}, Eqtemp(1), Eqtemp(2)};
    lbg = [lbg;   0;    0];
    ubg = [ubg; inf;  inf];

end

