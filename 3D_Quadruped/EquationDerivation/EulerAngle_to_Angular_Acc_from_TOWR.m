%% Derivatives of a Euler->Angular velocity mappings w.r.t euler angles.
%
% Used in class EulerConverter to specifcy analytical derivatives.
%
% Author: Alexander Winkler

clc;
clear all;

syms t % time and polynomial coefficients defining euler angles
syms phi(t) theta(t) psi(t) %euler angles yaw, pitch, roll 

% matrix that maps euler rates to angular velocities.
% see https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
C = [cos(theta)*cos(psi), -sin(psi), 0;
     cos(theta)*sin(psi),  cos(psi), 0;
     -sin(theta),              0,    1];


 
%% Derivative of M_dot

% Derivative of C w.r.t time
Cd = diff(C,t)

Cd_Thetad = Cd*[diff(phi,t);diff(theta,t);diff(psi,t)]


