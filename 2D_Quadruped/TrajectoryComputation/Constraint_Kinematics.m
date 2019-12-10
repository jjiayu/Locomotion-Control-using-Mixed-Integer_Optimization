function [g, lbg, ubg] = Constraint_Kinematics(Px,Pz,x,z,theta,k,PCenter,bw,bh)
%CONSTRAINT_KINEMATICS Kinematics Constraint

    g = {};
    lbg = [];
    ubg = [];
    
    Eqtemp = [cos(theta(k)), sin(theta(k)); -sin(theta(k)), cos(theta(k))]*([Px(k);Pz(k)] - [x(k);z(k)]) - PCenter;
    g = {g{:}, Eqtemp(1), Eqtemp(2)};
    lbg = [lbg; -bw/2; -bh/2];
    ubg = [ubg;  bw/2;  bh/2];

end

