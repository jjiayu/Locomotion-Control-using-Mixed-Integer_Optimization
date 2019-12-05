function J = PlanarArmJacobian(l1,l2,theta1,theta2)
%2DARMJACOBIAN Summary of this function goes here
%   Detailed explanation goes here
J = [-l1*sin(theta1) - l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
      l1*cos(theta1) + l2*cos(theta1+theta2),  l2*cos(theta1+theta2)];
end
