function [g, lbg, ubg] = Constraint_FeetPosition(Pz, M_pos, C, height, k, PhaseIdx)
%CONSTRAINT_FEETPOSITION Part of Complementarity Constraint

    g = {};
    lbg = [];
    ubg = [];

    %Equation 1:
    Eqtemp = Pz(k) - height + M_pos*C(PhaseIdx);
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  -inf];
    ubg = [ubg;  M_pos];
    
    %Equation 2:
    Eqtemp = Pz(k) -height;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];

end

