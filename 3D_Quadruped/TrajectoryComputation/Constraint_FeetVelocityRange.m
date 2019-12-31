function [g, lbg, ubg] = Constraint_FeetVelocityRange(Pxdot,Pydot,Pzdot,xdot,ydot,zdot,RTheta,k,Vmax)
%CONSTRAINT_FEETVELORANGE Feet Speed Ranges

    g = {};
    lbg = [];
    ubg = [];
    
    Eqtemp = RTheta'*([Pxdot(k);Pydot(k);Pzdot(k)] - [xdot(k);ydot(k);zdot(k)]); %We need quantities in Robot Frame, so we calculate the transpose of the rotation matrix
    g = {g{:}, Eqtemp(1), Eqtemp(2), Eqtemp(3)};
    lbg = [lbg; -Vmax];
    ubg = [ubg;  Vmax];

end

