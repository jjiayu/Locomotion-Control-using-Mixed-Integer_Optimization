function [g,lbg,ubg] = Constraint_FeetVeloActivation(Pxdot,Pzdot,C,k,PhaseIdx,Mvel)
%CONSTRAINT_FEETVELOACTIVATION Feet Velocity Activation/Deactivation

    g = {};
    lbg = [];
    ubg = [];
    
    %Set 1:
    Eqtemp = [Pxdot(k);Pzdot(k)] + Mvel*C(PhaseIdx);
    g = {g{:},      Eqtemp(1),  Eqtemp(2)};
    lbg = [lbg;     -inf;       -inf];
    ubg = [ubg;     Mvel;       Mvel];
    
    %Set 2:
    Eqtemp = [Pxdot(k);Pzdot(k)] - Mvel*C(PhaseIdx);
    g = {g{:},  Eqtemp(1),  Eqtemp(2)};
    lbg = [lbg; -Mvel; -Mvel];
    ubg = [ubg;   inf;  inf ];

end

