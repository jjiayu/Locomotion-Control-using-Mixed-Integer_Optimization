% Main Script for Computing Motion Plans/Trajectories for 2D Quadruped
%   Two Functionalities:
%       (1) MINLP to discover gait
%       (2) Continuous Optimization with Fixed Gait (for fixed gait evaluation and initial guess investigation)

% Model Information/Problem Formualtion information is listed in a written
% document

clear;
clc;

diary off

addpath(pwd)

% Import CasADi related packages
import casadi.*

%==========================================================================
% Define Robot Parameters
%==========================================================================

% 1-> Load from FIle

% 2-> Specify Maunually

%==========================================================================
% Computation Loop
%==========================================================================

%--------------------------------------------------------------------------
% Set-up Variables
%--------------------------------------------------------------------------



% Set-up Variable Bounds

% Set-up Initial Guesses

% Set-up Constraints

% Set-up Cost Functions

% Extract Results

%--------------------------------------------------------------------------
% Finalizing
%--------------------------------------------------------------------------

disp('===================================================');
disp('All Experiments Finished')
disp('===================================================');

% Send Emails

