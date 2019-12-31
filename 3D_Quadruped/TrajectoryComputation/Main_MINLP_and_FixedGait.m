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
        J = 0; %Cost Function Initialization

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
                %    Left Front (lf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plfz, M_pos, Clf, TerrainModel(Plfx(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Left Hind (lh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Plhz, M_pos, Clh, TerrainModel(Plhx(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Front (rf)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prfz, M_pos, Crf, TerrainModel(Prfx(k)), k, PhaseIdx);
                g   = {g{:},g_temp{:}}; %Add to constraint container
                lbg = [lbg; lbg_temp];
                ubg = [ubg; ubg_temp];
                %    Right Hind (rh)
                [g_temp,lbg_temp, ubg_temp] = Constraint_FeetPosition(Prhz, M_pos, Crh, TerrainModel(Prhx(k)), k, PhaseIdx);
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
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plfx,Plfz,x,z,theta,k,PlfCenter,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp];                                   

            %    Left Hind (lh)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Plhx,Plhz,x,z,theta,k,PlhCenter,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp];

            %    Right Front (rf)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prfx,Prfz,x,z,theta,k,PrfCenter,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp]; 

            %    Right Hind (rh)
            [g_temp,lbg_temp, ubg_temp] = Constraint_Kinematics(Prhx,Prhz,x,z,theta,k,PrhCenter,BoundingBox_Width,BoundingBox_Height);
            g   = {g{:},g_temp{:}}; %Add to constraint container
            lbg = [lbg; lbg_temp];
            ubg = [ubg; ubg_temp]; 

        end

        %----------------------------------
        %   Switching Time Constraint
        %----------------------------------
        Phaselb = phase_lower_bound_portion*Tend;
        [g_temp,lbg_temp, ubg_temp] = Constraint_SwitchingTime(Ts,Phaselb,Tend);
        g   = {g{:},g_temp{:}}; %Add to constraint container
        lbg = [lbg; lbg_temp];
        ubg = [ubg; ubg_temp]; 

        %----------------------------------
        %   Task and Peiodicity Constraint
        %----------------------------------
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
if exist('Remote_Computing_Flag','var') == 1 && Remote_Computing_Flag == 2 %Remote Computing
    %Send email to notify the success
    split_cmdoutput = splitlines(cmdoutput);
    sendmail('Jiayi.Wang@ed.ac.uk',[split_cmdoutput{2},' is done~']);
end
