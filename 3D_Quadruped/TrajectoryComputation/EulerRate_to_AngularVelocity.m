function omega = EulerRate_to_AngularVelocity(phi,theta,psi,phidot,thetadot,psidot)
%EULERRATE_TO_ANGULARVELOCITY Summary of this function goes here
%   Detailed explanation goes here
    Thetadot = [phidot;  thetadot;  psidot];
    
    C = [cos(theta)*cos(psi),       -sin(psi),           0;...
         cos(theta)*sin(psi),        cos(psi),           0;...
           -sin(theta)                   0,              1];
       
    omega = C*Thetadot;
    
end

