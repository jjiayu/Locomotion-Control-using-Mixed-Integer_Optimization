function omega_k = EulerRate_to_AngularVelocity(phi,theta,psi,phidot,thetadot,psidot,k)
%EULERRATE_TO_ANGULARVELOCITY Summary of this function goes here
%   Detailed explanation goes here
    Thetadot_k = [phidot(k);  thetadot(k);  psidot(k)];
    
    C_k = [cos(theta(k))*cos(psi(k)),       -sin(psi(k)),           0;...
           cos(theta(k))*sin(psi(k)),        cos(psi(k)),           0;...
           -sin(theta(k))                   0,              1];
       
    omega_k = C_k*Thetadot_k;
    
end

