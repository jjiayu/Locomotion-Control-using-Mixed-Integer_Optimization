clc;
clear;

syms phi theta psi %roll pitch yaw

%Elementary Rotations

Rz = [cos(psi),     -sin(psi),      0;...
      sin(psi),      cos(psi),      0;...
             0,             0,      1];
         
Ry = [ cos(theta),           0,      sin(theta);...
                0,           1,               0;...
      -sin(theta),           0,      cos(theta)];
  
Rx = [          1,            0,               0;...
                0,     cos(phi),       -sin(phi);...
                0,     sin(phi),        cos(phi)];

%Euler Angle Applied in ZYX axis
      
Rzyx = Rz*Ry*Rx

%Rxyz = Rx*Ry*Rz