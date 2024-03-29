% Main Script for Computing Motion Plans/Trajectories for 3D Quadruped
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

run('ParameterSetUp.m')

%==========================================================================
% Computation Loop (Where the big for-loop comes in)
%   Smart Tricks can be done to setup common computation euqations and
%   change the ones need to be changed
%==========================================================================

for speedIdx = 1:length(SpeedList)
    
    %--------------------------------------------------------------------------    
    % Get Task Specifications
    %--------------------------------------------------------------------------
    speed = SpeedList(speedIdx);
    %Tend = Tend; %Needed if we want to change Tend
    
    for runIdx = 1:NumofRuns

        %--------------------------------------------------------------------------
        % Logging of the Computation
        %--------------------------------------------------------------------------
        diary off
        ExpLog_filename = strcat(['Speed-', num2str(speed), '-'], datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
        diary([ExpDirectory, '/', ExpLog_filename]);

        disp('====================================================');
        disp(['Gait Search for ', num2str(NumPhases),'Phases Motion(', num2str(NumLocalKnots),'Knots per Phase),',' Locomotion Speed ', num2str(speed), ' m/s', ' Round ', num2str(runIdx)])
        disp(['Experiment Directory: ', ExpDirectory])
        disp(['Exp Start Time: ',datestr(datetime('now'), 30)])
        disp('====================================================');
        
        %--------------------------------------------------------------------------
        % Set-up Variables
        %   Build Decision Variable List for Optimization and Variable Name List
        %   for future look-up
        %--------------------------------------------------------------------------

        run('VariableSetUp.m')

        %--------------------------------------------------------------------------
        % Set-up Constraints
        %--------------------------------------------------------------------------
        %   Initialise Constarint Constainers
        g   = {}; %Constraint Function Container
        lbg = []; %Lower Bound of Constraint Function
        ubg = []; %Upper Bound of Constraint Function

        %   Create Time Step Variables for each phase
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
            
            %----------------------------------
            % Get the Orientation of the Torso (Euler Angle to Rotation Matrix)
            %----------------------------------
            RTheta = EulerAngle_to_RotationMatrix(phi, theta, psi, k);
            %----------------------------------

            if k <= tauSeriesLength - 1
                %----------------------------------
                %Build Constraint for System Dynamics
                %----------------------------------
                % Generate Constraints
                [g_temp,lbg_temp, ubg_temp] = Constraint_SystemDynamics(m,         I,        G,...           %Inertia Info
                                                                        x,         y,         z,...           %Linear Position
                                                                        xdot,      ydot,      zdot,...        %Linear Velocity
                                                                        phi,       theta,     psi,...         %Orientation
                                                                        phidot,    thetadot,  psidot,...      %Euler Angle Rate
                                                                        Plfx,      Plfy,      Plfz,...        %Left Front Feet Location (lf)
                                                                        Plhx,      Plhy,      Plhz,...        %Left Hind Feet Location (lh)
                                                                        Prfx,      Prfy,      Prfz,...        %Right Front Feet Location (rf)
                                                                        Prhx,      Prhy,      Prhz,...        %Right Hind Feet Location (rh)
                                                                        Plfxdot,   Plfydot,   Plfzdot,...     %Left Front Feet Velocity (lf)
                                                                        Plhxdot,   Plhydot,   Plhzdot,...     %Left Hind Feet Velocity (lh)
                                                                        Prfxdot,   Prfydot,   Prfzdot,...     %Right Front Feet Velocity (rf)
                                                                        Prhxdot,   Prhydot,   Prhzdot,...     %Right Hind Feet Velocity (rh)
                                                                        Flfx,      Flfy,      Flfz,...        %Left Front Feet Force (lf)
                                                                        Flhx,      Flhy,      Flhz,...        %Left Hind Feet Force (lh)
                                                                        Frfx,      Frfy,      Frfz,...        %Right Front Feet Force (rf)
                                                                        Frhx,      Frhy,      Frhz,...        %Right Hind Feet Force (rh)
                                                                        h,...                             %No Switching time vector, instead put Time Steps
                                                                        RTheta,...                        %Rotated Matrix Converted from Euler Angles
                                                                        k);                               %Knot Number

                                                                    
                % Add to the constraint container
                g   = {g{:},g_temp{:}};
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %----------------------------------
                %Build Complementarity Constraint
                %----------------------------------
                %   Feet Position
                %----------------------------------
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plfz, M_pos, Clf, TerrainModel(Plfx(k),Plfy(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plhz, M_pos, Clh, TerrainModel(Plhx(k),Plhy(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prfz, M_pos, Crf, TerrainModel(Prfx(k),Prfy(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prhz, M_pos, Crh, TerrainModel(Prhx(k),Prhy(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %----------------------------------
                %   Feet Velocity Activation/Deactivation
                %----------------------------------
                %!!Mvel is a column vector
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plfxdot,Plfydot,Plfzdot,Clf,k,PhaseIdx,Mvel);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Plhxdot,Plhydot,Plhzdot,Clh,k,PhaseIdx,Mvel);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prfxdot,Prfydot,Prfzdot,Crf,k,PhaseIdx,Mvel);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVeloActivation(Prhxdot,Prhydot,Prhzdot,Crh,k,PhaseIdx,Mvel);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %----------------------------------
                %   Feet Velocity Ranges
                %----------------------------------
                %!! Vmax is a columbn vector
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Plfxdot,Plfydot,Plfzdot,xdot,ydot,zdot,RTheta,k,Vmax);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Plhxdot,Plhydot,Plhzdot,xdot,ydot,zdot,RTheta,k,Vmax);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Prfxdot,Prfydot,Prfzdot,xdot,ydot,zdot,RTheta,k,Vmax);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetVelocityRange(Prhxdot,Prhydot,Prhzdot,xdot,ydot,zdot,RTheta,k,Vmax);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %----------------------------------
                %   Force Activation/Deactivation
                %----------------------------------
                %!! Mf is a column vector
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Flfx,Flfy,Flfz,Clf,k,PhaseIdx,terrain_slope_rad,Mf);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Flhx,Flhy,Flhz,Clh,k,PhaseIdx,terrain_slope_rad,Mf);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Frfx,Frfy,Frfz,Crf,k,PhaseIdx,terrain_slope_rad,Mf);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_ForceActivation(Frhx,Frhy,Frhz,Crh,k,PhaseIdx,terrain_slope_rad,Mf);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %----------------------------------
                %   Unilateral Constraint of (Foot-Ground Reaction Forces)
                %----------------------------------
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Flfx,Flfy,Flfz,k,TerrainNorm);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Flhx,Flhy,Flhz,k,TerrainNorm);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Frfx,Frfy,Frfz,k,TerrainNorm);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_UnilateralForce(Frhx,Frhy,Frhz,k,TerrainNorm);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

                %----------------------------------
                %   Friction Cone
                %----------------------------------
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Flfx,Flfy,Flfz,k,TerrainTangentX,TerrainTangentY,TerrainNorm,miu);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Flhx,Flhy,Flhz,k,TerrainTangentX,TerrainTangentY,TerrainNorm,miu);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Frfx,Frfy,Frfz,k,TerrainTangentX,TerrainTangentY,TerrainNorm,miu);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FrictionCone(Frhx,Frhy,Frhz,k,TerrainTangentX,TerrainTangentY,TerrainNorm,miu);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];

            end

            %----------------------------------
            %   Kinematics Constraint
            %----------------------------------
            %    Left Front (lf)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plfx,Plfy,Plfz,x,y,z,RTheta,k,PlfCenter,BoundingBox_Length,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp];                                   

            %    Left Hind (lh)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plhx,Plhy,Plhz,x,y,z,RTheta,k,PlhCenter,BoundingBox_Length,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp];

            %    Right Front (rf)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prfx,Prfy,Prfz,x,y,z,RTheta,k,PrfCenter,BoundingBox_Length,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp]; 

            %    Right Hind (rh)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prhx,Prhy,Prhz,x,y,z,RTheta,k,PrhCenter,BoundingBox_Length,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp]; 

        end

        %----------------------------------
        %   Switching Time Constraint
        %----------------------------------
        %Phaselb = phase_lower_bound_portion*Tend;
        [g_temp,lbg_temp, ubg_temp] = Constraint_SwitchingTime(Ts,Phaselb,Tend);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp]; 

        %----------------------------------
        %   Task and Peiodicity Constraint
        %----------------------------------
        [g_temp,lbg_temp, ubg_temp] = Constraint_Task_and_Periodicity(x,         y,         z,...           %Linear Position
                                                                      xdot,      ydot,      zdot,...        %Linear Velocity
                                                                      phi,       theta,     psi,...         %Orientation
                                                                      phidot,    thetadot,  psidot,...      %Euler Angle Rate
                                                                      Plfx,      Plfy,      Plfz,...        %Left Front Feet Location (lf)
                                                                      Plhx,      Plhy,      Plhz,...        %Left Hind Feet Location (lh)
                                                                      Prfx,      Prfy,      Prfz,...        %Right Front Feet Location (rf)
                                                                      Prhx,      Prhy,      Prhz,...        %Right Hind Feet Location (rh)
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
        J = 0; %Cost Function Initialization
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
                J = J + h*(Flfx(k)^2) + h*(Flfy(k)^2) + h*(Flfz(k)^2) +... Left Front (lf)
                        h*(Flhx(k)^2) + h*(Flhy(k)^2) + h*(Flhz(k)^2) +... Left Hind (lh)
                        h*(Frfx(k)^2) + h*(Frfy(k)^2) + h*(Frfz(k)^2) +... Right Front (rf)
                        h*(Frhx(k)^2) + h*(Frhy(k)^2) + h*(Frhz(k)^2);     %Right Hind (rh)
            elseif cost_flag == 2 %Minimize Force Squared + Mini Body Vibration
                if SpeedDirection == 1 %along x-axis in the world frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed/cos(terrain_slope_rad))*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2) + ...
                            h*((Flfx(k)/10)^2) + h*((Flfy(k)/10)^2) + h*((Flfz(k)/10)^2) +... Left Front (lf)
                            h*((Flhx(k)/10)^2) + h*((Flhy(k)/10)^2) + h*((Flhz(k)/10)^2) +... Left Hind (lh)
                            h*((Frfx(k)/10)^2) + h*((Frfy(k)/10)^2) + h*((Frfz(k)/10)^2) +... Right Front (rf)
                            h*((Frhx(k)/10)^2) + h*((Frhy(k)/10)^2) + h*((Frhz(k)/10)^2);     %Right Hind (rh)
                    
                elseif SpeedDirection == 2 %along x-axis in the terrain frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed)*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2) + ...
                            h*((Flfx(k)/10)^2) + h*((Flfy(k)/10)^2) + h*((Flfz(k)/10)^2) +... Left Front (lf)
                            h*((Flhx(k)/10)^2) + h*((Flhy(k)/10)^2) + h*((Flhz(k)/10)^2) +... Left Hind (lh)
                            h*((Frfx(k)/10)^2) + h*((Frfy(k)/10)^2) + h*((Frfz(k)/10)^2) +... Right Front (rf)
                            h*((Frhx(k)/10)^2) + h*((Frhy(k)/10)^2) + h*((Frhz(k)/10)^2);     %Right Hind (rh)
                end

%                 J = J + h*(((xdot(k)-speed)*1e3)^2) + ...
%                         h*((y(k)*1e3)^2) + ...
%                         h*((ydot(k)*1e3)^2) + ...
%                         h*((zdot(k)*1e3)^2) + ...
%                         h*((phi(k)*1e3)^2)  + ...
%                         h*((phidot(k)*1e3)^2) + ...
%                         h*((theta(k)*1e3)^2) + ...
%                         h*((thetadot(k)*1e3)^2) + ...
%                         h*((psi(k)*1e3)^2) + ...
%                         h*((psidot(k)*1e3)^2) + ...
%                         h*((Flfx(k)/10)^2) + h*((Flfy(k)/10)^2) + h*((Flfz(k)/10)^2) +... Left Front (lf)
%                         h*((Flhx(k)/10)^2) + h*((Flhy(k)/10)^2) + h*((Flhz(k)/10)^2) +... Left Hind (lh)
%                         h*((Frfx(k)/10)^2) + h*((Frfy(k)/10)^2) + h*((Frfz(k)/10)^2) +... Right Front (rf)
%                         h*((Frhx(k)/10)^2) + h*((Frhy(k)/10)^2) + h*((Frhz(k)/10)^2);     %Right Hind (rh)
            elseif cost_flag == 3 %MiniBodyVibration
                if SpeedDirection == 1 %along x-axis in the world frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed/cos(terrain_slope_rad))*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2);
                elseif SpeedDirection == 2 %along x-axis in the terrain frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed)*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2);
                end
                
%                 J = J + h*(((xdot(k)-speed)*1e3)^2) + ...
%                         h*((y(k)*1e3)^2) + ...
%                         h*((ydot(k)*1e3)^2) + ...
%                         h*((zdot(k)*1e3)^2) + ...
%                         h*((phi(k)*1e3)^2)  + ...
%                         h*((phidot(k)*1e3)^2) + ...
%                         h*((theta(k)*1e3)^2) + ...
%                         h*((thetadot(k)*1e3)^2) + ...
%                         h*((psi(k)*1e3)^2) + ...
%                         h*((psidot(k)*1e3)^2);
                    
            elseif cost_flag == 4 %Smoothness of Force Profile
                if k<=tauSeriesLength - 2
                    J = J + (Flfx(k+1)-Flfx(k))^2/h + (Flfy(k+1)-Flfy(k))^2/h + (Flfz(k+1)-Flfz(k))^2/h +... Left Front (lf)
                            (Flhx(k+1)-Flhx(k))^2/h + (Flhy(k+1)-Flhy(k))^2/h + (Flhz(k+1)-Flhz(k))^2/h +... Left Hind (lh)
                            (Frfx(k+1)-Frfx(k))^2/h + (Frfy(k+1)-Frfy(k))^2/h + (Frfz(k+1)-Frfz(k))^2/h +... Right Front (rf)
                            (Frhx(k+1)-Frhx(k))^2/h + (Frhy(k+1)-Frhy(k))^2/h + (Frhz(k+1)-Frhz(k))^2/h;    %Right Hind (rh)
                end
            elseif cost_flag == 5 %Minimize Body Vibration with Minimize limb y-axis deviaton from nominal positons
                %Limb Locations in robot frame
                Plf_Robot = RTheta'*[Plfx(k);Plfy(k);Plfz(k)];
                Plh_Robot = RTheta'*[Plhx(k);Plhy(k);Plhz(k)];
                Prf_Robot = RTheta'*[Prfx(k);Prfy(k);Prfz(k)];
                Prh_Robot = RTheta'*[Prhx(k);Prhy(k);Prhz(k)];
                
                if SpeedDirection == 1 %along x-axis in the world frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed/cos(terrain_slope_rad))*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2) + ...
                            h*(((Plf_Robot(2) - PlfCenter(2))*1e3)^2) + ...
                            h*(((Plh_Robot(2) - PlhCenter(2))*1e3)^2) + ...
                            h*(((Prf_Robot(2) - PrfCenter(2))*1e3)^2) + ...
                            h*(((Prh_Robot(2) - PrhCenter(2))*1e3)^2);
                    
                elseif SpeedDirection == 2 %along x-axis in the terrain frame
                    J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed)*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2) + ...
                            h*(((Plf_Robot(2) - PlfCenter(2))*1e3)^2) + ...
                            h*(((Plh_Robot(2) - PlhCenter(2))*1e3)^2) + ...
                            h*(((Prf_Robot(2) - PrfCenter(2))*1e3)^2) + ...
                            h*(((Prh_Robot(2) - PrhCenter(2))*1e3)^2);
                end

%                 J = J + h*(((xdot(k)-speed)*1e3)^2) + ...
%                         h*((y(k)*1e3)^2) + ...
%                         h*((ydot(k)*1e3)^2) + ...
%                         h*((zdot(k)*1e3)^2) + ...
%                         h*((phi(k)*1e3)^2)  + ...
%                         h*((phidot(k)*1e3)^2) + ...
%                         h*((theta(k)*1e3)^2) + ...
%                         h*((thetadot(k)*1e3)^2) + ...
%                         h*((psi(k)*1e3)^2) + ...
%                         h*((psidot(k)*1e3)^2) +...
%                         h*(((Plf_Robot(2) - PlfCenter(2))*1e3)^2) + ...
%                         h*(((Plh_Robot(2) - PlhCenter(2))*1e3)^2) + ...
%                         h*(((Prf_Robot(2) - PrfCenter(2))*1e3)^2) + ...
%                         h*(((Prh_Robot(2) - PrhCenter(2))*1e3)^2);
                
            elseif cost_flag == 6 %Feet Velocity (Pending) with cost 2
                        J = J + h*((([xdot(k),ydot(k),zdot(k)]*TerrainTangentX - speed)*1e3)^2) + ...
                            h*((y(k)*1e3)^2) + ...
                            h*((ydot(k)*1e3)^2) + ...
                            h*((([xdot(k),ydot(k),zdot(k)]*TerrainNorm)*1e3)^2) + ...
                            h*((phi(k)*1e3)^2)  + ...
                            h*((phidot(k)*1e3)^2) + ...
                            h*(((theta(k)-(-terrain_slope_rad))*1e3)^2) + ...
                            h*((thetadot(k)*1e3)^2) + ...
                            h*((psi(k)*1e3)^2) + ...
                            h*((psidot(k)*1e3)^2) + ...
                            h*((Flfx(k)/10)^2) + h*((Flfy(k)/10)^2) + h*((Flfz(k)/10)^2) +... Left Front (lf)
                            h*((Flhx(k)/10)^2) + h*((Flhy(k)/10)^2) + h*((Flhz(k)/10)^2) +... Left Hind (lh)
                            h*((Frfx(k)/10)^2) + h*((Frfy(k)/10)^2) + h*((Frfz(k)/10)^2) +... Right Front (rf)
                            h*((Frhx(k)/10)^2) + h*((Frhy(k)/10)^2) + h*((Frhz(k)/10)^2) +... Right Hind (rh)
                            h*(((Plfx(k)-PlfCenter(1))*1e3)^2) + ...
                            h*(((Plhx(k)-PlhCenter(1))*1e3)^2) + ...
                            h*(((Prfx(k)-PrfCenter(1))*1e3)^2) + ...
                            h*(((Prhx(k)-PrhCenter(1))*1e3)^2);
                        
            end
        end
        %----------------------------------
        disp('=====================================================');
        disp('All Constraint Set up')
        disp('=====================================================');
        disp(' ')

        %--------------------------------------------------------------------------
        % Set-up Variable Bounds
        %--------------------------------------------------------------------------
        run('BuildVarBounds.m')

        disp('=====================================================');
        disp('Boundaries of Decision Variables Build')
        disp('=====================================================');
        disp(' ')

        %--------------------------------------------------------------------------
        % Set-up Initial Guesses
        %--------------------------------------------------------------------------
        run('BuildInitialGuess.m')

        disp('=====================================================');
        disp('Initial Guess Build')
        disp('=====================================================');
        disp(' ')

        %--------------------------------------------------------------------------
        % Call the Optimization
        %--------------------------------------------------------------------------
        disp('===================================================')
        disp('Optimization Start')
        disp('===================================================')
        disp(' ')

        run('CallSolver.m')

        %--------------------------------------------------------------------------
        % Extract Results
        %--------------------------------------------------------------------------
        
        run('ResultExtraction.m')
        
        disp('===================================================')
        disp('Computation Result Extracted')
        disp('===================================================')
        disp(' ')
        
        
        %--------------------------------------------------------------------------
        % Close Diary
        %--------------------------------------------------------------------------
        diary off
    end
end
%--------------------------------------------------------------------------
% Finalizing the Computation
%--------------------------------------------------------------------------

disp('===================================================');
disp('All Experiments Finished')
disp('===================================================');

% Send Emails

if Eddie_flag ~= 1 %Not using Eddie, then do this
    if exist('Remote_Computing_Flag','var') == 1 && Remote_Computing_Flag == 2 %Remote Computing
        %Send email to notify the success
        split_cmdoutput = splitlines(cmdoutput);
        sendmail('Jiayi.Wang@ed.ac.uk',[split_cmdoutput{2},' is done~']);
    end
end

InfCompute_flag = str2double(getenv('InfCompute'));
if Eddie_flag == 1 && InfCompute_flag == 1 %Not using Eddie, then do this
    if exist('Remote_Computing_Flag','var') == 1 && Remote_Computing_Flag == 2 %Remote Computing
        %Send email to notify the success
        split_cmdoutput = splitlines(cmdoutput);
        sendmail('Jiayi.Wang@ed.ac.uk',[split_cmdoutput{2},' is done~']);
    end
end