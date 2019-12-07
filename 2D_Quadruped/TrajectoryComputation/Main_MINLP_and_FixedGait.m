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
%   Build Decision Variable List for Optimization and Variable Name List
%   for future look-up
%--------------------------------------------------------------------------
%!!!!!!!!!!!!!!!!!
%Temp Parameters -> Remove When finish
%!!!!!!!!!!!!!!!!!
m = 21;
Iyy = 2.3;
G = 9.8;
tauSeriesLength = 41
NumPhases = 4
gait_discovery_switch = 1

run('VariableSetUp.m')

%--------------------------------------------------------------------------
% Set-up Variable Bounds
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Set-up Initial Guesses
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Set-up Constraints
%--------------------------------------------------------------------------
%   Initialise Constarint Constainers
g   = {}; %Constraint Function Container
lbg = []; %Lower Bound of Constraint Function
ubg = []; %Upper Bound of Constraint Function

%   Create Time Step Variables for each phase
% %!!!!!!!!!!!!!!!!!!! Temp Variable -> Remove when Finish
tauStepLength = 0.1;
NumLocalKnots = 10;
% %!!!!!!!!!!!!!!!!!!!
PhaseDurationVector = [Ts(1)-0;diff(Ts)];
hVector = tauStepLength*NumPhases*PhaseDurationVector;

%   Task/Scenario-Independent Constraints
%       Problem Formulations that do not change with respect to the change
%       of task/scenario specifications

for k = 1:tauSeriesLength
    
    %--------------------------------------
    % Extract Phase Index
    %--------------------------------------
    if k <= tauSeriesLength - 1
        PhaseIdx = floor((k-1)/NumLocalKnots) + 1; % k-1 is the time step enumeration
    elseif k == tauSeriesLength
        PhaseIdx = floor((k-1)/NumLocalKnots) + 1 - 1; %the equation will give last time belongs to NumPhases +1, we simply classify it into the last phase
    end
    %--------------------------------------

    %----------------------------------
    % Get time step
    %----------------------------------
    h = hVector(PhaseIdx);
    %----------------------------------
    
    if k <= tauSeriesLength - 1
        %----------------------------------
        %Build Constraint for System Dynamics
        %----------------------------------
        % Generate Constraints
        [g_temp,lbg_temp, ubg_temp] = Constraint_SystemDynamics(m,         Iyy,        G,...           %Inertia Info
                                                      x,         z,        theta,...     %Decision Variables
                                                      xdot,      zdot,     thetadot,...
                                                      Plfx,      Plfz,...
                                                      Plhx,      Plhz,...
                                                      Prfx,      Prfz,...
                                                      Prhx,      Prhz,...
                                                      Plfxdot,   Plfzdot,...
                                                      Plhxdot,   Plhzdot,...
                                                      Prfxdot,   Prfzdot,...
                                                      Prhxdot,   Prhzdot,...
                                                      Flfx,      Flfz,...
                                                      Flhx,      Flhz,...
                                                      Frfx,      Frfz,...
                                                      Frhx,      Frhz,...
                                                      h,...                             %No Switching time vector, instead put Time Steps
                                                      k);                                %Knot Number
        % Add to the constraint container
        g   = {g{:},g_temp{:}};
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %----------------------------------
        %Build Complementarity Constraint
        %----------------------------------
        %   Feet Position
        %----------------------------------
        %!!!!!!!!!!!!!
        %Change to function -> height(x)
        height = 0;
        %!!!!!!!!!!!!!
        %Temporary Parameter
        M_pos = 100;
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plfz, M_pos, Clf, height, k, PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plhz, M_pos, Clh, height, k, PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prfz, M_pos, Crf, height, k, PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prhz, M_pos, Crh, height, k, PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %----------------------------------
        %   Feet Velocity Activation/Deactivation
        %----------------------------------
        %!!!!!!!!!!!!!
        %Temporary Parameter
        Mvel = 100;
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plfxdot,Plfzdot,Clf,Mvel,k,PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plhxdot,Plhzdot,Clh,Mvel,k,PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prfxdot,Prfzdot,Crf,Mvel,k,PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prhxdot,Prhzdot,Crh,Mvel,k,PhaseIdx);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %----------------------------------
        
        
    end
    
                                              

    
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Set-up Cost Functions
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Extract Results
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Finalizing the Computation
%--------------------------------------------------------------------------

disp('===================================================');
disp('All Experiments Finished')
disp('===================================================');

% Send Emails

