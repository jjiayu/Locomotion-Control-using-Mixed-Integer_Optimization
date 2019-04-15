% Mixed-integer Nonlinear Optimization in 2D case, use auto differentiation
% to compute gradients and hessians
% Mode Selection at every time step
% Foot-ground contact configurations are modeled as 0/1 for each foot

% Check readme for notes ad future improvements

clear;
clc;

%========================================================
%Command Line Logging
diary off
diary_filename = strcat('log-', datestr(datetime('now'),30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
diary(diary_filename);

%=========================================================
%Display Script Information
disp('Date and Time:');
disp(datetime('now'));
disp(['Correspondent Log File Name: ', diary_filename]);
disp(' ');
disp('2D Locomotion Control using Mixed-integer Nonlinear Optimization')
disp('Mode Configuration Change per Time Step')
disp(' ')

%=====================================================================
% Add path
%addpath('/home/jiayu/bin/casadi-linux-matlabR2014b-v3.4.5')

%=====================================================================
% Choose solver

solverNum = input('Select Solver First, 1: Knitro, 2: Bonmin \n');
if solverNum == 1
    solverSelected = 'knitro';
elseif solverNum == 2
    solverSelected = 'bonmin';
else
    ME_SelectSolvers = MException('Initialization:SelectSolvers','Unknown Solver Nominated');
    throw(ME_SelectSolvers)
end
%======================================================================
%Inertia Parameters(Information from MIT Cheetah 3)
m = 45; %kg
I = 2.1; %kg m^2 Izz
g = 9.80665; %m/s^2
%======================================================================

%======================================================================
%Environment Information
TerrainHeight = 0; %terrain height
TerrainNorm = [0,1];
miu = 0.6; %friction coefficient
%======================================================================

%======================================================================
% Time Step and Discretization Parameter Settings




