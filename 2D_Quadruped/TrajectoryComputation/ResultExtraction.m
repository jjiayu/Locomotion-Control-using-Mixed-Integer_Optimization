%   Extract the Computation Results

disp('===================================================');
disp('Computation Result Extraction:');
disp('---------------------------------------------------');
%-----------------------------------------------------------------------
%   Recover the full solution
%-----------------------------------------------------------------------
res = full(sol.x);

%   Extract Switching Time
PhaseSwitchingTime = [0;res(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end)))]; %include initial time 0
TimeSlope = NumPhases.*diff(PhaseSwitchingTime);
TimeIntercept = PhaseSwitchingTime(1:end-1);

TimeSeries = zeros(NumKnots + 1,1); %with starting time 0
for i = 1:NumPhases
    for j = 1:NumLocalKnots
        TimeSeries((i-1)*NumLocalKnots + j + 1) = TimeIntercept(i) + TimeSlope(i)*j*tauStepLength; %start from index 2, index 1 is time 0
    end
end
%------------------------------------------------------------------------

%------------------------------------------------------------------------
%   Extract Original Solutions/Variables
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Robot state (position)
%------------------------------------------------------------------------
x_result     = res(find(VarNamesList == 'x_0'):find(VarNamesList == x_label(end)));
z_result     = res(find(VarNamesList == 'z_0'):find(VarNamesList == z_label(end)));
theta_result = res(find(VarNamesList == 'theta_0'):find(VarNamesList == theta_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Robot state (Velocity)
%------------------------------------------------------------------------
xdot_result = res(find(VarNamesList == 'xdot_0'):find(VarNamesList == xdot_label(end)));
zdot_result = res(find(VarNamesList == 'zdot_0'):find(VarNamesList == zdot_label(end)));
thetadot_result = res(find(VarNamesList == 'thetadot_0'):find(VarNamesList == thetadot_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% End-effector locations
%------------------------------------------------------------------------
%   Left Front (lf)
Plfx_result = res(find(VarNamesList == 'Plfx_0'):find(VarNamesList == Plfx_label(end)));
Plfz_result = res(find(VarNamesList == 'Plfz_0'):find(VarNamesList == Plfz_label(end)));
%   Left Hind (lh)
Plhx_result = res(find(VarNamesList == 'Plhx_0'):find(VarNamesList == Plhx_label(end)));
Plhz_result = res(find(VarNamesList == 'Plhz_0'):find(VarNamesList == Plhz_label(end)));
%   Right Front (rf)
Prfx_result = res(find(VarNamesList == 'Prfx_0'):find(VarNamesList == Prfx_label(end)));
Prfz_result = res(find(VarNamesList == 'Prfz_0'):find(VarNamesList == Prfz_label(end)));
%   Right Hind (rh)
Prhx_result = res(find(VarNamesList == 'Prhx_0'):find(VarNamesList == Prhx_label(end)));
Prhz_result = res(find(VarNamesList == 'Prhz_0'):find(VarNamesList == Prhz_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% End-effector velocities
%------------------------------------------------------------------------
%   Left Front (lf)
Plfxdot_result = res(find(VarNamesList == 'Plfxdot_0'):find(VarNamesList == Plfxdot_label(end)));
Plfzdot_result = res(find(VarNamesList == 'Plfzdot_0'):find(VarNamesList == Plfzdot_label(end)));
%   Left Hind (lh)
Plhxdot_result = res(find(VarNamesList == 'Plhxdot_0'):find(VarNamesList == Plhxdot_label(end)));
Plhzdot_result = res(find(VarNamesList == 'Plhzdot_0'):find(VarNamesList == Plhzdot_label(end)));
%   Right Front (rf)
Prfxdot_result = res(find(VarNamesList == 'Prfxdot_0'):find(VarNamesList == Prfxdot_label(end)));
Prfzdot_result = res(find(VarNamesList == 'Prfzdot_0'):find(VarNamesList == Prfzdot_label(end)));
%   Right Hind (rh)
Prhxdot_result = res(find(VarNamesList == 'Prhxdot_0'):find(VarNamesList == Prhxdot_label(end)));
Prhzdot_result = res(find(VarNamesList == 'Prhzdot_0'):find(VarNamesList == Prhzdot_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Default Foot Location in WORLD Frame
%------------------------------------------------------------------------
%   Left Front (lf)
PlfxCenter_result_world = x_result + cos(theta_result)*PlfCenter(1) - sin(theta_result)*PlfCenter(2);
PlfzCenter_result_world = z_result + sin(theta_result)*PlfCenter(1) + cos(theta_result)*PlfCenter(2);
%   Left Hind (lh)
PlhxCenter_result_world = x_result + cos(theta_result)*PlhCenter(1) - sin(theta_result)*PlhCenter(2);
PlhzCenter_result_world = z_result + sin(theta_result)*PlhCenter(1) + cos(theta_result)*PlhCenter(2);
%   Right Front (rf)
PrfxCenter_result_world = x_result + cos(theta_result)*PrfCenter(1) - sin(theta_result)*PrfCenter(2);
PrfzCenter_result_world = z_result + sin(theta_result)*PrfCenter(1) + cos(theta_result)*PrfCenter(2);
%   Right Hind (rh)
PrhxCenter_result_world = x_result + cos(theta_result)*PrhCenter(1) - sin(theta_result)*PrhCenter(2);
PrhzCenter_result_world = z_result + sin(theta_result)*PrhCenter(1) + cos(theta_result)*PrhCenter(2);

%------------------------------------------------------------------------
% Contact force result
%------------------------------------------------------------------------
%---------------------
% Contact Forces for Individual Limbs
%---------------------
%   Left Front (lf)
Flfx_result = res(find(VarNamesList == 'Flfx_0'):find(VarNamesList == Flfx_label(end)));
Flfz_result = res(find(VarNamesList == 'Flfz_0'):find(VarNamesList == Flfz_label(end)));
%   Left Hind (lh)
Flhx_result = res(find(VarNamesList == 'Flhx_0'):find(VarNamesList == Flhx_label(end)));
Flhz_result = res(find(VarNamesList == 'Flhz_0'):find(VarNamesList == Flhz_label(end)));
%   Right Front (rf)
Frfx_result = res(find(VarNamesList == 'Frfx_0'):find(VarNamesList == Frfx_label(end)));
Frfz_result = res(find(VarNamesList == 'Frfz_0'):find(VarNamesList == Frfz_label(end)));
%   Right Hind (rh)
Frhx_result = res(find(VarNamesList == 'Frhx_0'):find(VarNamesList == Frhx_label(end)));
Frhz_result = res(find(VarNamesList == 'Frhz_0'):find(VarNamesList == Frhz_label(end)));
%---------------------
% Compute Net Forces
%---------------------
%           Left Front(lf) Left Hind(lh) Right Front(rf) Right Hind(rh)
NetForceX = Flfx_result + Flhx_result + Frfx_result + Frhx_result;
NetForceZ = Flfz_result + Flhz_result + Frfz_result + Frhz_result;
%---------------------
% Net Torques on the Body
%---------------------
%   Torque from Left Front (lf)
LeftFrontTorque_result = (Plfx_result(1:end-1) - x_result(1:end-1)).*Flfz_result - (Plfz_result(1:end-1) - z_result(1:end-1)).*Flfx_result;
%   Torque from Left Hind (lh)
LeftHindTorque_result = (Plhx_result(1:end-1) - x_result(1:end-1)).*Flhz_result - (Plhz_result(1:end-1) - z_result(1:end-1)).*Flhx_result;
%   Torque from Right Front (rf)
RightFrontTorque_result = (Prfx_result(1:end-1) - x_result(1:end-1)).*Frfz_result - (Prfz_result(1:end-1) - z_result(1:end-1)).*Frfx_result;
%   Torque from Right Hind (rh)
RightHindTorque_result = (Prhx_result(1:end-1) - x_result(1:end-1)).*Frhz_result - (Prhz_result(1:end-1) - z_result(1:end-1)).*Frhx_result;
%   Net Torque
NetTorque = LeftFrontTorque_result + LeftHindTorque_result + RightFrontTorque_result + RightHindTorque_result;

%------------------------------------------------------------------------
% Contact Configuration
%------------------------------------------------------------------------
if gait_discovery_switch == 1 %Free Gait Discovery using MINLP
    %   Left Front (lf)
    Clf_result = res(find(VarNamesList == Clf_label(1)):find(VarNamesList == Clf_label(end)));
    %   Left Hind (lh)
    Clh_result = res(find(VarNamesList == Clh_label(1)):find(VarNamesList == Clh_label(end)));
    %   Right Front (rf)
    Crf_result = res(find(VarNamesList == Crf_label(1)):find(VarNamesList == Crf_label(end)));
    %   Right Hind (rh)
    Crh_result = res(find(VarNamesList == Crh_label(1)):find(VarNamesList == Crh_label(end)));
elseif gait_discovery_switch == 2 %Fixed Gait Optimization
    %   Left Front (lf)
    Clf_result = Clf;
    %   Left Hind (lh)
    Clh_result = Clh;
    %   Right Front (rf)
    Crf_result = Crf;
    %   Right Hind (rh)
    Crh_result = Crh;
end

disp('Result Variables Extracted')

%---------------------------------------------------------------------
% Backup Original Results
%---------------------------------------------------------------------
%--------------------
%   Time Quantities
%--------------------
%       Time Series
TimeSeries_origin  = TimeSeries;
%       Phase Switching Time/Terminal Time
Ts_origin = res(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end)));
%--------------------
%   Robot Body Trajectories
%--------------------
x_result_origin    = x_result;       z_result_origin    = z_result;      theta_result_origin    = theta_result;
xdot_result_origin = xdot_result;    zdot_result_origin = zdot_result;   thetadot_result_origin = thetadot_result;
%--------------------
%   Feet Locations
%--------------------
%       Left Front (lf)
Plfx_result_origin = Plfx_result;    Plfz_result_origin = Plfz_result;
%       Left Hind (lh)
Plhx_result_origin = Plhx_result;    Plhz_result_origin = Plhz_result;
%       Right Front (rf)
Prfx_result_origin = Prfx_result;    Prfz_result_origin = Prfz_result;
%       Right Hind (rh)
Prhx_result_origin = Prhx_result;    Prhz_result_origin = Prhz_result;
%--------------------
%   Feet Velocities
%--------------------
%       Left Front (lf)
Plfxdot_result_origin = Plfxdot_result;    Plfzdot_result_origin = Plfzdot_result;
%       Left Hind (lh)
Plhxdot_result_origin = Plhxdot_result;    Plhzdot_result_origin = Plhzdot_result;
%       Right Front (rf)
Prfxdot_result_origin = Prfxdot_result;    Prfzdot_result_origin = Prfzdot_result;
%       Right Hind (rh)
Prhxdot_result_origin = Prhxdot_result;    Prhzdot_result_origin = Prhzdot_result;
%--------------------
%   Default Feet Locations in World Frame
%--------------------
%       Left Front (lf)
PlfxCenter_result_world_origin = PlfxCenter_result_world;     PlfzCenter_result_world_origin = PlfzCenter_result_world;
%       Left Hind (lh)
PlhxCenter_result_world_origin = PlhxCenter_result_world;     PlhzCenter_result_world_origin = PlhzCenter_result_world;
%       Right Front (rf)
PrfxCenter_result_world_origin = PrfxCenter_result_world;     PrfzCenter_result_world_origin = PrfzCenter_result_world;
%       Right Hind (rh)
PrhxCenter_result_world_origin = PrhxCenter_result_world;     PrhzCenter_result_world_origin = PrhzCenter_result_world;
%--------------------
%   Feet Forces
%--------------------
%       Left Front (lf)
Flfx_result_origin = Flfx_result;    Flfz_result_origin = Flfz_result;
%       Left Hind (lh)
Flhx_result_origin = Flhx_result;    Flhz_result_origin = Flhz_result;
%       Right Front (rf)
Frfx_result_origin = Frfx_result;    Frfz_result_origin = Frfz_result;
%       Right Hind (rh)
Frhx_result_origin = Frhx_result;    Frhz_result_origin = Frhz_result;
%       Net Forces
NetForceX_origin   = NetForceX;      NetForceZ_origin   = NetForceZ;
%       Torques on the Torso
LeftFrontTorque_result_origin = LeftFrontTorque_result;      
LeftHindTorque_result_origin = LeftHindTorque_result;
RightFrontTorque_result_origin = RightFrontTorque_result;
RightHindTorque_result_origin = RightHindTorque_result;
NetTorque_origin   = NetTorque;
%--------------------
%   Contact Configuration
%--------------------
%       Left Front (lf)
Clf_result_origin = Clf_result;
%       Left Hind (lh)
Clh_result_origin = Clh_result;
%       Right Front (rf)
Crf_result_origin = Crf_result;
%       Right Hind (rh)
Crh_result_origin = Crh_result;
disp('Original Result Variables Backuped - Result Include Vanishing Phases - End with "origin"');
%-----------------------------------------------------------------------

%-----------------------------------------------------------------------
% Clean Up Time/Control/State Lists - Remove Phases with Zero Length  
%-----------------------------------------------------------------------
TimeStepDiff = diff(TimeSeries);

%First-oprder State index, TimeStepDiff + 1
%---------------
%   Time Series
%---------------
TimeSeries(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Robot Torso State
%---------------
x_result(find(TimeStepDiff <= 1e-3) + 1) = [];
z_result(find(TimeStepDiff <= 1e-3) + 1) = [];
theta_result(find(TimeStepDiff <= 1e-3) + 1) = [];

xdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
zdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
thetadot_result(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Location
%---------------
%       Left Front (lf)
Plfx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plfz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Left Hind (lh)
Plhx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plhz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Front (rf)
Prfx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prfz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Hind (rh)
Prhx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prhz_result(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Velocity
%---------------
%       Left Front (lf)
Plfxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Plfzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Left Hind (lh)
Plhxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Plhzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Front (rf)
Prfxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Prfzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Hind (rh)
Prhxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Prhzdot_result(find(TimeStepDiff <= 1e-3)) = [];

%---------------
%   Default Feet Location
%---------------
%       Left Front (lf)
PlfxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlfzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Left Hind (lh)
PlhxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlhzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Front (rf)
PrfxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrfzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Hind (rh)
PrhxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrhzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Forces
%---------------
%       Left Front (lf)
Flfx_result(find(TimeStepDiff <= 1e-3)) = [];
Flfz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Left Hind (lh)
Flhx_result(find(TimeStepDiff <= 1e-3)) = [];
Flhz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Front (rf)
Frfx_result(find(TimeStepDiff <= 1e-3)) = [];
Frfz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Hind (rh)
Frhx_result(find(TimeStepDiff <= 1e-3)) = [];
Frhz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Net Forces
NetForceX(find(TimeStepDiff <= 1e-3)) = [];
NetForceZ(find(TimeStepDiff <= 1e-3)) = [];
%       Net Torque
NetTorque(find(TimeStepDiff <= 1e-3)) = [];

%---------------
%Phase Lengths
%---------------
PhaseLengths_origin = diff(PhaseSwitchingTime);
PhaseLengths = PhaseLengths_origin;
PhaseLengths(find(PhaseLengths_origin <= 1e-3 )) = [];

%---------------
%Contact Configurations
%---------------
%       Left Front (lf)
Clf_result(find(PhaseLengths_origin <= 1e-3 )) = [];
%       Left Hind (lh)
Clh_result(find(PhaseLengths_origin <= 1e-3 )) = [];
%       Right Front (rf)
Crf_result(find(PhaseLengths_origin <= 1e-3 )) = [];
%       Right Hind (rh)
Crh_result(find(PhaseLengths_origin <= 1e-3 )) = [];


% Build Gait Matrix

gait = [Clf_result,...
        Clh_result,...
        Crf_result,...
        Crh_result,...
        PhaseLengths];

disp('Removed Variables within Vanished Phases');
disp('===================================================');
disp(' ');
%=======================================================================
%retrieve cost as well
result_cost = full(sol.f); %cost function

% Save Experimental Result
warning('off')
save([ExpDirectory, '/', ExpLog_filename, '.mat']);
warning('on')
