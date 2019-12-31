function [g,lbg,ubg] = Constraint_ForceActivation(Fx,Fy,Fz,C,k,PhaseIdx,theta_slope,Mf)
%CONSTRAINT_FORCEACTIVATION Activation and Deactivation of Forces Due to
%complementarity constraint

    g = {};
    lbg = [];
    ubg = [];
    
    Rslope = ElementaryRotation_Y(-theta_slope);
    %REVERSE the rotated angle, because slope up need to rotated negative
    %angles with repsect to y-axis in "3D case"
    %slope down need to rotated positive angles with respect to y-axis in
    %"3D case"
    
    %Set 1:
    Eqtemp = Rslope'*[Fx(k);Fy(k);Fz(k)] - Mf*C(PhaseIdx);
    g = {g{:}, Eqtemp(1), Eqtemp(2), Eqtemp(3)};
    lbg = [lbg; -inf; -inf; -inf];
    ubg = [ubg;    0;    0;    0];
    
    %Set 2:
    Eqtemp = Rslope'*[Fx(k);Fy(k);Fz(k)] + Mf*C(PhaseIdx);
    g = {g{:}, Eqtemp(1), Eqtemp(2), Eqtemp(3)};
    lbg = [lbg;   0;    0;    0];
    ubg = [ubg; inf;  inf;  inf];

end

