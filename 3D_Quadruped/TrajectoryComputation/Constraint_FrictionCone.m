function [g, lbg, ubg] = Constraint_FrictionCone(Fx,Fz,k,TerrainTangent,TerrainNorm,miu)
%CONSTRAINT_FRICTIONCONE Constraint for building friction cones

    
    g = {};
    lbg = [];
    ubg = [];
    
    %Set 1:
    Eqtemp = [Fx(k),Fz(k)]*TerrainTangent - miu*[Fx(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  -inf];
    ubg = [ubg;  0];
    
    %Set 2:
    Eqtemp = [Fx(k),Fz(k)]*TerrainTangent + miu*[Fx(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];

end

