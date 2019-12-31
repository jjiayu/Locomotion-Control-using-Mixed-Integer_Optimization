function [g,lbg,ubg] = Constraint_UnilateralForce(Fx,Fy,Fz,k,TerrainNorm)
%CONSTRAINT_UNILATERALFORCE Unilateral Constraint
    g = {};
    lbg = [];
    ubg = [];
    
    Eqtemp = [Fx(k),Fy(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];

end

