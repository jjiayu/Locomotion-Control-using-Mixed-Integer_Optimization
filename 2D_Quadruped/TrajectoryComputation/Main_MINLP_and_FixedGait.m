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
% Set-up Constraints
%--------------------------------------------------------------------------
%   Initialise Constarint Constainers
g   = {}; %Constraint Function Container
lbg = []; %Lower Bound of Constraint Function
ubg = []; %Upper Bound of Constraint Function
J = 0; %Cost Function Initialization

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
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plfxdot,Plfzdot,Clf,k,PhaseIdx,Mvel);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plhxdot,Plhzdot,Clh,k,PhaseIdx,Mvel);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prfxdot,Prfzdot,Crf,k,PhaseIdx,Mvel);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prhxdot,Prhzdot,Crh,k,PhaseIdx,Mvel);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %----------------------------------
        %   Feet Velocity Ranges
        %----------------------------------
        %!!!!!!!!!!!!!
        %Temporary Parameter
        Vmax = 100;
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Plfxdot,Plfzdot,xdot,zdot,theta,k,Vmax);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        

        
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Plhxdot,Plhzdot,xdot,zdot,theta,k,Vmax);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Prfxdot,Prfzdot,xdot,zdot,theta,k,Vmax);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Prhxdot,Prhzdot,xdot,zdot,theta,k,Vmax);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %----------------------------------
        %   Force Activation/Deactivation
        %----------------------------------
                %!!!!!!!!!!!!!
        %Temporary Parameter
        Mf = [150;250];
        terrain_slope_rad = 20/180*pi;
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Flfx,Flfz,Clf,k,PhaseIdx,terrain_slope_rad,Mf);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Flhx,Flhz,Clh,k,PhaseIdx,terrain_slope_rad,Mf);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Frfx,Frfz,Crf,k,PhaseIdx,terrain_slope_rad,Mf);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Frhx,Frhz,Crh,k,PhaseIdx,terrain_slope_rad,Mf);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %----------------------------------
        %   Unilateral Constraint of (Foot-Ground Reaction Forces)
        %----------------------------------
        %!!!!!!!!!!!!!
        %Temporary Parameter
        TerrainNorm = [0;1];
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Flfx,Flfz,k,TerrainNorm);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Flhx,Flhz,k,TerrainNorm);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Frfx,Frfz,k,TerrainNorm);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Frhx,Frhz,k,TerrainNorm);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
        %----------------------------------
        %   Friction Cone
        %----------------------------------
        %!!!!!!!!!!!!!
        %Temporary Parameter
        miu = 0.6;
        TerrainTangent = [1;0];
        %!!!!!!!!!!!!!
        %    Left Front (lf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Flfx,Flfz,k,TerrainTangent,TerrainNorm,miu);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Left Hind (lh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Flhx,Flhz,k,TerrainTangent,TerrainNorm,miu);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Front (rf)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Frfx,Frfz,k,TerrainTangent,TerrainNorm,miu);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        %    Right Hind (rh)
        [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Frhx,Frhz,k,TerrainTangent,TerrainNorm,miu);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp];
        
    end
    
    %----------------------------------
    %   Kinematics Constraint
    %----------------------------------
    %!!!!!!!!!!!!!
    %Temporary Parameter
    PlfCenter = [0;1];
    PlhCenter = [1;1];
    PrfCenter = [1;1.5];
    PrhCenter = [2;1];
    BoundingBoxWidth = 0.4;
    BoundingBoxHeight = 0.4;
    %!!!!!!!!!!!!!
    %    Left Front (lf)
    [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plfx,Plfz,x,z,theta,k,PlfCenter,BoundingBoxWidth,BoundingBoxHeight);
    g   = {g{:},g_temp{:}}; %Add to constraint container
    lbg = [lbg; lbg_temp];
    ubg = [ubg; ubg_temp];                                   

    %    Left Hind (lh)
    [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plhx,Plhz,x,z,theta,k,PlhCenter,BoundingBoxWidth,BoundingBoxHeight);
    g   = {g{:},g_temp{:}}; %Add to constraint container
    lbg = [lbg; lbg_temp];
    ubg = [ubg; ubg_temp];
    
    %    Right Front (rf)
    [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prfx,Prfz,x,z,theta,k,PrfCenter,BoundingBoxWidth,BoundingBoxHeight);
    g   = {g{:},g_temp{:}}; %Add to constraint container
    lbg = [lbg; lbg_temp];
    ubg = [ubg; ubg_temp]; 
    
    %    Right Hind (rh)
    [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prhx,Prhz,x,z,theta,k,PrhCenter,BoundingBoxWidth,BoundingBoxHeight);
    g   = {g{:},g_temp{:}}; %Add to constraint container
    lbg = [lbg; lbg_temp];
    ubg = [ubg; ubg_temp]; 

end

%----------------------------------
%   Switching Time Constraint
%----------------------------------
%!!!!!!!!!!!!!
%Temporary Parameter
Tend = 0.4;
Phaselb = Tend*10/100;%Percentage Portion
%!!!!!!!!!!!!!
[g_temp,lbg_temp, ubg_temp] = Constraint_SwitchingTime(Ts,Phaselb,Tend);
g   = {g{:},g_temp{:}}; %Add to constraint container
lbg = [lbg; lbg_temp];
ubg = [ubg; ubg_temp]; 

%----------------------------------
%   Task and Peiodicity Constraint
%----------------------------------
%!!!!!!!!!!!!!
%Temporary Parameter
speed = 0.4;
SpeedDirection = 1;%Percentage Portion
%!!!!!!!!!!!!!
[g_temp,lbg_temp, ubg_temp] = Constraint_Task_and_Periodicity(x,         z,        theta,...     %Decision Variables
                                                              xdot,      zdot,     thetadot,...
                                                              Plfx,      Plfz,...
                                                              Plhx,      Plhz,...
                                                              Prfx,      Prfz,...
                                                              Prhx,      Prhz,...
                                                              Clf,...
                                                              Clh,...
                                                              Crf,...
                                                              Crh,...
                                                              Ts,        Tend,...
                                                              speed,     SpeedDirection,...
                                                              terrain_slope_rad);
g   = {g{:},g_temp{:}}; %Add to constraint container
lbg = [lbg; lbg_temp];
ubg = [ubg; ubg_temp]; 

%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Set-up Cost Functions
%--------------------------------------------------------------------------
%!!!!!!!!!!!!!
%Temporary Parameter
cost_flag = 5;
%!!!!!!!!!!!!!
Scale_Factor = 1000; %difference below 1e-3 are treated as the same
for k = 1:tauSeriesLength - 1
    
    %--------------------------------------
    % Extract Phase Index
    %--------------------------------------
    PhaseIdx = floor((k-1)/NumLocalKnots) + 1; % k-1 is the time step enumeration
    %--------------------------------------
    %----------------------------------
    % Get time step
    %----------------------------------
    h = hVector(PhaseIdx);
    %----------------------------------
    
    if cost_flag == 1 %Minimize Force Squared (Energy Loss)
        J = J + h*(Flfx(k)^2) + h*(Flfz(k)^2) + ... Left Front (lf)
                h*(Flhx(k)^2) + h*(Flhz(k)^2) + ... Left Hind (lh)
                h*(Frfx(k)^2) + h*(Frfz(k)^2) + ... Right Front (rf)
                h*(Frhx(k)^2) + h*(Frhz(k)^2);     %Right Hind (rh)
    elseif cost_flag == 2 %Minimize Tangential Force (Maximize Robustness)
        error('No.2 Cost Not Implemented');
    elseif cost_flag == 3 %Minimize Vibration (theta towards terrain slope, thetadot towards zero, normal velocity towards zero)
        % Time Integral and Scaled
        J = J + h*(((theta(k)-terrain_slope_rad)*Scale_Factor)^2) + ...    theta towards terrain slope
                h*((thetadot(k)*Scale_Factor)^2) + ...                     thetadot towards zero
                h*((([xdot(k),zdot(k)]*TerrainNorm)*Scale_Factor)^2);     %normal velocity towards zero
    elseif cost_flag == 4 %5 -> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)
        error('No.4 Cost is Redundant');
    elseif cost_flag == 5 %Minimize Vibration (Cost 3) with Constant Tangential Velocity (at Every Knot)
        % Add Constant Tangential Velocity (at Every Knot) Term -> Depends on which direction the desired speed is defined
        if SpeedDirection == 1 %speed is defind along horizontal direction
            %A simpler form
            %J = J + h*(((xdot(k)-speed)*Scale_Factor)^2)
            J = J + h*((([xdot(k),zdot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2);
        elseif SpeedDirection == 2 %speed is defined along tangential direction
            J = J + h*((([xdot(k),zdot(k)]*TerrainTangent - speed)*Scale_Factor)^2);
        end
        % Build Cost First -> Time Integral and Scaled
        J = J + h*(((theta(k)-terrain_slope_rad)*Scale_Factor)^2) + ...    theta towards terrain slope
                h*((thetadot(k)*Scale_Factor)^2) + ...                     thetadot towards zero
                h*((([xdot(k),zdot(k)]*TerrainNorm)*Scale_Factor)^2);     %normal velocity towards zero
    elseif cost_flag == 6 %Feet Velocity (Pending)
        error('No.6 Cost is not implemented')
    end
end
%----------------------------------

%--------------------------------------------------------------------------
% Set-up Variable Bounds
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Set-up Initial Guesses
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

