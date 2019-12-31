function [g, lbg, ubg] = Constraint_Kinematics(Px,Py,Pz,x,y,z,RTheta,k,PCenter,bl,bw,bh)
%CONSTRAINT_KINEMATICS Kinematics Constraint

    g = {};
    lbg = [];
    ubg = [];
    
    Eqtemp = RTheta'*([Px(k);Py(k);Pz(k)] - [x(k);y(k);z(k)]) - PCenter; %Transpose of the Rotation Matrix because we want quantities in robot frame
    g = {g{:}, Eqtemp(1), Eqtemp(2), Eqtemp(3)};
    lbg = [lbg; -bl/2; -bw/2; -bh/2];
    ubg = [ubg;  bl/2;  bw/2;  bh/2];

end

