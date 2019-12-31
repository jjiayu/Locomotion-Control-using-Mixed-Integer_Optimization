function [g, lbg, ubg] = Constraint_FrictionCone(Fx,Fy,Fz,k,TerrainTangentX,TerrainTangentY,TerrainNorm,miu)
%CONSTRAINT_FRICTIONCONE Constraint for building friction cones

    
    g = {};
    lbg = [];
    ubg = [];
    
    %Group 1: Tangential X-axis along the terrain
    %   Set 1:
    Eqtemp = [Fx(k),Fy(k),Fz(k)]*TerrainTangentX - miu*[Fx(k),Fy(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  -inf];
    ubg = [ubg;  0];
    %   Set 2:
    Eqtemp = [Fx(k),Fy(k),Fz(k)]*TerrainTangentX + miu*[Fx(k),Fy(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];
    
    %Group 2: Tangential Y-axis along the terrain
    %   Set 1:
    Eqtemp = [Fx(k),Fy(k),Fz(k)]*TerrainTangentY - miu*[Fx(k),Fy(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  -inf];
    ubg = [ubg;  0];
    %   Set 2:
    Eqtemp = [Fx(k),Fy(k),Fz(k)]*TerrainTangentY + miu*[Fx(k),Fy(k),Fz(k)]*TerrainNorm;
    g   = {g{:}, Eqtemp};
    lbg = [lbg;  0];
    ubg = [ubg;  inf];

end

