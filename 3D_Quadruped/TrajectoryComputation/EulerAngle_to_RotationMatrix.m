function RTheta = EulerAngle_to_RotationMatrix(phi, theta, psi, k)
%EULERANGLE_TO_ROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here

%   Elementary Rotations
%       around z-axis
Rz = [cos(psi(k)),     -sin(psi(k)),        0;...
      sin(psi(k)),      cos(psi(k)),        0;...
                0,                0,        1];
         
%       around y-axis
Ry = [ cos(theta(k)),           0,      sin(theta(k));...
                   0,           1,                  0;...
      -sin(theta(k)),           0,      cos(theta(k))];
  
%       around x-axis
Rx = [          1,               0,                  0;...
                0,     cos(phi(k)),       -sin(phi(k));...
                0,     sin(phi(k)),        cos(phi(k))];
            
%   Euler Angle ZYX sequence
RTheta = Rz*Ry*Rx;

end

