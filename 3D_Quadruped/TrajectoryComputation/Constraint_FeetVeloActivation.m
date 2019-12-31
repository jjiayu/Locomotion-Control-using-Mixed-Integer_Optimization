function [g,lbg,ubg] = Constraint_FeetVeloActivation(Pxdot,Pydot,Pzdot,C,k,PhaseIdx,Mvel)
%CONSTRAINT_FEETVELOACTIVATION Feet Velocity Activation/Deactivation

    g = {};
    lbg = [];
    ubg = [];
    
    %Set 1:
    Eqtemp = [Pxdot(k);Pydot(k);Pzdot(k)] + Mvel*C(PhaseIdx);
    g = {g{:},      Eqtemp(1),      Eqtemp(2),      Eqtemp(3)};
    lbg = [lbg;     -inf;           -inf;           -inf];
    ubg = [ubg;     Mvel;           Mvel;           Mvel];%Mvel is just a very big Value (single-dimensional), so we need to duplicate for 3 times
    
    %Set 2:
    Eqtemp = [Pxdot(k);Pydot(k);Pzdot(k)] - Mvel*C(PhaseIdx);
    g = {g{:},      Eqtemp(1),      Eqtemp(2),      Eqtemp(3)};
    lbg = [lbg;     -Mvel;          -Mvel;          -Mvel];%Mvel is just a very big Value (single-dimensional), so we need to duplicate for 3 time
    ubg = [ubg;     inf;            inf;            inf];

end

