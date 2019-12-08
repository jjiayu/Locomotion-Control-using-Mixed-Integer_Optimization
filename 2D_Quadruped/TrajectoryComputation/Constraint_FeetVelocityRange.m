function [g, lbg, ubg] = Constraint_FeetVelocityRange(Pxdot,Pzdot,xdot,zdot,theta,k,Vmax)
%CONSTRAINT_FEETVELORANGE Feet Speed Ranges

    g = {};
    lbg = [];
    ubg = [];
    
    Eqtemp = [cos(theta(k)), sin(theta(k)); -sin(theta(k)), cos(theta(k))]*([Pxdot(k);Pzdot(k)] - [xdot(k);zdot(k)]);
    g = {g{:}, Eqtemp(1), Eqtemp(2)};
    lbg = [lbg; -Vmax; -Vmax];
    ubg = [ubg;  Vmax;  Vmax];

end

