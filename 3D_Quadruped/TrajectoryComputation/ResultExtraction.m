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
% Robot state (Linear Position)
%------------------------------------------------------------------------
x_result     = res(find(VarNamesList == 'x_0'):find(VarNamesList == x_label(end)));
y_result     = res(find(VarNamesList == 'y_0'):find(VarNamesList == y_label(end)));
z_result     = res(find(VarNamesList == 'z_0'):find(VarNamesList == z_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Robot state (Linear Velocity)
%------------------------------------------------------------------------
xdot_result = res(find(VarNamesList == 'xdot_0'):find(VarNamesList == xdot_label(end)));
ydot_result = res(find(VarNamesList == 'ydot_0'):find(VarNamesList == ydot_label(end)));
zdot_result = res(find(VarNamesList == 'zdot_0'):find(VarNamesList == zdot_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Orientation (Eular Angles)
%------------------------------------------------------------------------
phi_result   = res(find(VarNamesList == 'phi_0'):find(VarNamesList == phi_label(end)));
theta_result = res(find(VarNamesList == 'theta_0'):find(VarNamesList == theta_label(end)));
psi_result   = res(find(VarNamesList == 'psi_0'):find(VarNamesList == psi_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Euler Angle Rates
%------------------------------------------------------------------------
phidot_result   = res(find(VarNamesList == 'phidot_0'):find(VarNamesList == phidot_label(end)));
thetadot_result = res(find(VarNamesList == 'thetadot_0'):find(VarNamesList == thetadot_label(end)));
psidot_result   = res(find(VarNamesList == 'psidot_0'):find(VarNamesList == psidot_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% End-effector locations
%------------------------------------------------------------------------
%   Left Front (lf)
Plfx_result = res(find(VarNamesList == 'Plfx_0'):find(VarNamesList == Plfx_label(end)));
Plfy_result = res(find(VarNamesList == 'Plfy_0'):find(VarNamesList == Plfy_label(end)));
Plfz_result = res(find(VarNamesList == 'Plfz_0'):find(VarNamesList == Plfz_label(end)));
%   Left Hind (lh)
Plhx_result = res(find(VarNamesList == 'Plhx_0'):find(VarNamesList == Plhx_label(end)));
Plhy_result = res(find(VarNamesList == 'Plhy_0'):find(VarNamesList == Plhy_label(end)));
Plhz_result = res(find(VarNamesList == 'Plhz_0'):find(VarNamesList == Plhz_label(end)));
%   Right Front (rf)
Prfx_result = res(find(VarNamesList == 'Prfx_0'):find(VarNamesList == Prfx_label(end)));
Prfy_result = res(find(VarNamesList == 'Prfy_0'):find(VarNamesList == Prfy_label(end)));
Prfz_result = res(find(VarNamesList == 'Prfz_0'):find(VarNamesList == Prfz_label(end)));
%   Right Hind (rh)
Prhx_result = res(find(VarNamesList == 'Prhx_0'):find(VarNamesList == Prhx_label(end)));
Prhy_result = res(find(VarNamesList == 'Prhy_0'):find(VarNamesList == Prhy_label(end)));
Prhz_result = res(find(VarNamesList == 'Prhz_0'):find(VarNamesList == Prhz_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% End-effector velocities
%------------------------------------------------------------------------
%   Left Front (lf)
Plfxdot_result = res(find(VarNamesList == 'Plfxdot_0'):find(VarNamesList == Plfxdot_label(end)));
Plfydot_result = res(find(VarNamesList == 'Plfydot_0'):find(VarNamesList == Plfydot_label(end)));
Plfzdot_result = res(find(VarNamesList == 'Plfzdot_0'):find(VarNamesList == Plfzdot_label(end)));
%   Left Hind (lh)
Plhxdot_result = res(find(VarNamesList == 'Plhxdot_0'):find(VarNamesList == Plhxdot_label(end)));
Plhydot_result = res(find(VarNamesList == 'Plhydot_0'):find(VarNamesList == Plhydot_label(end)));
Plhzdot_result = res(find(VarNamesList == 'Plhzdot_0'):find(VarNamesList == Plhzdot_label(end)));
%   Right Front (rf)
Prfxdot_result = res(find(VarNamesList == 'Prfxdot_0'):find(VarNamesList == Prfxdot_label(end)));
Prfydot_result = res(find(VarNamesList == 'Prfydot_0'):find(VarNamesList == Prfydot_label(end)));
Prfzdot_result = res(find(VarNamesList == 'Prfzdot_0'):find(VarNamesList == Prfzdot_label(end)));
%   Right Hind (rh)
Prhxdot_result = res(find(VarNamesList == 'Prhxdot_0'):find(VarNamesList == Prhxdot_label(end)));
Prhydot_result = res(find(VarNamesList == 'Prhydot_0'):find(VarNamesList == Prhydot_label(end)));
Prhzdot_result = res(find(VarNamesList == 'Prhzdot_0'):find(VarNamesList == Prhzdot_label(end)));
%------------------------------------------------------------------------

%------------------------------------------------------------------------
% Default Foot Location in WORLD Frame
%------------------------------------------------------------------------
PlfxCenter_result_world = zeros(size(theta_result)); PlfyCenter_result_world = zeros(size(theta_result)); PlfzCenter_result_world = zeros(size(theta_result));
PlhxCenter_result_world = zeros(size(theta_result)); PlhyCenter_result_world = zeros(size(theta_result)); PlhzCenter_result_world = zeros(size(theta_result));
PrfxCenter_result_world = zeros(size(theta_result)); PrfyCenter_result_world = zeros(size(theta_result)); PrfzCenter_result_world = zeros(size(theta_result));
PrhxCenter_result_world = zeros(size(theta_result)); PrhyCenter_result_world = zeros(size(theta_result)); PrhzCenter_result_world = zeros(size(theta_result));

for ResKnotIdx = 1:length(theta_result)
    RTheta_k_result = EulerAngle_to_RotationMatrix(phi_result, theta_result, psi_result, ResKnotIdx);
    r_result_k = [x_result(ResKnotIdx);y_result(ResKnotIdx);z_result(ResKnotIdx)]; %torso result at times step KnotIdx
    
    PlfCenter_k_result_world =  r_result_k + RTheta_k_result*PlfCenter;
    PlhCenter_k_result_world =  r_result_k + RTheta_k_result*PlhCenter;
    PrfCenter_k_result_world =  r_result_k + RTheta_k_result*PrfCenter;
    PrhCenter_k_result_world =  r_result_k + RTheta_k_result*PrhCenter;
    
    %   Left Front (lf)
    PlfxCenter_result_world(ResKnotIdx) = PlfCenter_k_result_world(1);
    PlfyCenter_result_world(ResKnotIdx) = PlfCenter_k_result_world(2);
    PlfzCenter_result_world(ResKnotIdx) = PlfCenter_k_result_world(3);

    %   Left Hind (lh)
    PlhxCenter_result_world(ResKnotIdx) = PlhCenter_k_result_world(1);
    PlhyCenter_result_world(ResKnotIdx) = PlhCenter_k_result_world(2);
    PlhzCenter_result_world(ResKnotIdx) = PlhCenter_k_result_world(3);
    
    %   Right Front (rf)
    PrfxCenter_result_world(ResKnotIdx) = PrfCenter_k_result_world(1);
    PrfyCenter_result_world(ResKnotIdx) = PrfCenter_k_result_world(2);
    PrfzCenter_result_world(ResKnotIdx) = PrfCenter_k_result_world(3);
    
    %   Right Hind (rh)
    PrhxCenter_result_world(ResKnotIdx) = PrhCenter_k_result_world(1);
    PrhyCenter_result_world(ResKnotIdx) = PrhCenter_k_result_world(2);
    PrhzCenter_result_world(ResKnotIdx) = PrhCenter_k_result_world(3);
end

%------------------------------------------------------------------------
% Contact force result
%------------------------------------------------------------------------
%---------------------
% Contact Forces for Individual Limbs
%---------------------
%   Left Front (lf)
Flfx_result = res(find(VarNamesList == 'Flfx_0'):find(VarNamesList == Flfx_label(end)));
Flfy_result = res(find(VarNamesList == 'Flfy_0'):find(VarNamesList == Flfy_label(end)));
Flfz_result = res(find(VarNamesList == 'Flfz_0'):find(VarNamesList == Flfz_label(end)));
%   Left Hind (lh)
Flhx_result = res(find(VarNamesList == 'Flhx_0'):find(VarNamesList == Flhx_label(end)));
Flhy_result = res(find(VarNamesList == 'Flhy_0'):find(VarNamesList == Flhy_label(end)));
Flhz_result = res(find(VarNamesList == 'Flhz_0'):find(VarNamesList == Flhz_label(end)));
%   Right Front (rf)
Frfx_result = res(find(VarNamesList == 'Frfx_0'):find(VarNamesList == Frfx_label(end)));
Frfy_result = res(find(VarNamesList == 'Frfy_0'):find(VarNamesList == Frfy_label(end)));
Frfz_result = res(find(VarNamesList == 'Frfz_0'):find(VarNamesList == Frfz_label(end)));
%   Right Hind (rh)
Frhx_result = res(find(VarNamesList == 'Frhx_0'):find(VarNamesList == Frhx_label(end)));
Frhy_result = res(find(VarNamesList == 'Frhy_0'):find(VarNamesList == Frhy_label(end)));
Frhz_result = res(find(VarNamesList == 'Frhz_0'):find(VarNamesList == Frhz_label(end)));
%---------------------
% Compute Net Forces
%---------------------
%           Left Front(lf) Left Hind(lh) Right Front(rf) Right Hind(rh)
NetForceX = Flfx_result + Flhx_result + Frfx_result + Frhx_result;
NetForceY = Flfy_result + Flhy_result + Frfy_result + Frhy_result;
NetForceZ = Flfz_result + Flhz_result + Frfz_result + Frhz_result;

%---------------------
% Net Torques on the Body
%---------------------
LeftFrontTorqueX_result = zeros(size(Flfx_result));
LeftFrontTorqueY_result = zeros(size(Flfx_result));
LeftFrontTorqueZ_result = zeros(size(Flfx_result));

LeftHindTorqueX_result = zeros(size(Flfx_result));
LeftHindTorqueY_result = zeros(size(Flfx_result));
LeftHindTorqueZ_result = zeros(size(Flfx_result));

RightFrontTorqueX_result = zeros(size(Flfx_result));
RightFrontTorqueY_result = zeros(size(Flfx_result));
RightFrontTorqueZ_result = zeros(size(Flfx_result));

RightHindTorqueX_result = zeros(size(Flfx_result));
RightHindTorqueY_result = zeros(size(Flfx_result));
RightHindTorqueZ_result = zeros(size(Flfx_result));

for ResKnotIdx = 1:length(theta_result)-1 %NumKnot-1 is the length of system inputs
    r_result_k = [x_result(ResKnotIdx);y_result(ResKnotIdx);z_result(ResKnotIdx)]; %torso result at times step KnotIdx
    Plf_result_k = [Plfx_result(ResKnotIdx); Plfy_result(ResKnotIdx); Plfz_result(ResKnotIdx)];
    Plh_result_k = [Plhx_result(ResKnotIdx); Plhy_result(ResKnotIdx); Plhz_result(ResKnotIdx)];
    Prf_result_k = [Prfx_result(ResKnotIdx); Prfy_result(ResKnotIdx); Prfz_result(ResKnotIdx)];
    Prh_result_k = [Prhx_result(ResKnotIdx); Prhy_result(ResKnotIdx); Prhz_result(ResKnotIdx)];
    
    Flf_result_k = [Flfx_result(ResKnotIdx); Flfy_result(ResKnotIdx); Flfz_result(ResKnotIdx)];
    Flh_result_k = [Flhx_result(ResKnotIdx); Flhy_result(ResKnotIdx); Flhz_result(ResKnotIdx)];
    Frf_result_k = [Frfx_result(ResKnotIdx); Frfy_result(ResKnotIdx); Frfz_result(ResKnotIdx)];
    Frh_result_k = [Frhx_result(ResKnotIdx); Frhy_result(ResKnotIdx); Frhz_result(ResKnotIdx)];
    
    LeftFrontTorque_k = cross((Plf_result_k-r_result_k),Flf_result_k);
    LeftHindTorque_k= cross((Plh_result_k-r_result_k),Flh_result_k);
    RightFrontTorque_k = cross((Prf_result_k-r_result_k),Frf_result_k);
    RightHindTorque_k = cross((Prh_result_k-r_result_k),Frh_result_k);
    
    LeftFrontTorqueX_result(ResKnotIdx) = LeftFrontTorque_k(1);
    LeftFrontTorqueY_result(ResKnotIdx) = LeftFrontTorque_k(2);
    LeftFrontTorqueZ_result(ResKnotIdx) = LeftFrontTorque_k(3);
    
    LeftHindTorqueX_result(ResKnotIdx) = LeftHindTorque_k(1);
    LeftHindTorqueY_result(ResKnotIdx) = LeftHindTorque_k(2);
    LeftHindTorqueZ_result(ResKnotIdx) = LeftHindTorque_k(3);
    
    RightFrontTorqueX_result(ResKnotIdx) = RightFrontTorque_k(1);
    RightFrontTorqueY_result(ResKnotIdx) = RightFrontTorque_k(2);
    RightFrontTorqueZ_result(ResKnotIdx) = RightFrontTorque_k(3);
    
    RightHindTorqueX_result(ResKnotIdx) = RightHindTorque_k(1);
    RightHindTorqueY_result(ResKnotIdx) = RightHindTorque_k(2);
    RightHindTorqueZ_result(ResKnotIdx) = RightHindTorque_k(3);
end

%   Net Torque
%NetTorque = LeftFrontTorque_result + LeftHindTorque_result + RightFrontTorque_result + RightHindTorque_result;

NetTorqueX = LeftFrontTorqueX_result + LeftHindTorqueX_result + RightFrontTorqueX_result + RightHindTorqueX_result;
NetTorqueY = LeftFrontTorqueY_result + LeftHindTorqueY_result + RightFrontTorqueY_result + RightHindTorqueY_result;
NetTorqueZ = LeftFrontTorqueZ_result + LeftHindTorqueZ_result + RightFrontTorqueZ_result + RightHindTorqueZ_result;

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

% Intial Angular Velocity of the Body
omega0 = EulerRate_to_AngularVelocity(phi_result,theta_result,psi_result,phidot_result,thetadot_result,psidot_result,1);


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
%   Robot Body (Linear Position)
%--------------------
x_result_origin    = x_result;      y_result_origin     = y_result;      z_result_origin    = z_result;
%--------------------
%   Robot Body (Linear Velocity)
%--------------------
xdot_result_origin = xdot_result;   ydot_result_origin  = ydot_result;   zdot_result_origin = zdot_result;   
%--------------------
%   Robot Body (Orientation)
%--------------------
phi_result_origin    = phi_result;      theta_result_origin     = theta_result;      psi_result_origin    = psi_result;
%--------------------
%   Robot Body (Euler Angle Rates)
%--------------------
phidot_result_origin    = phidot_result;      thetadot_result_origin     = thetadot_result;      psidot_result_origin    = psidot_result;
%--------------------
%   Feet Locations
%--------------------
%       Left Front (lf)
Plfx_result_origin = Plfx_result;    Plfy_result_origin = Plfy_result;      Plfz_result_origin = Plfz_result;
%       Left Hind (lh)
Plhx_result_origin = Plhx_result;    Plhy_result_origin = Plhy_result;      Plhz_result_origin = Plhz_result;
%       Right Front (rf)
Prfx_result_origin = Prfx_result;    Prfy_result_origin = Prfy_result;      Prfz_result_origin = Prfz_result;
%       Right Hind (rh)
Prhx_result_origin = Prhx_result;    Prhy_result_origin = Prhy_result;      Prhz_result_origin = Prhz_result;
%--------------------
%   Feet Velocities
%--------------------
%       Left Front (lf)
Plfxdot_result_origin = Plfxdot_result;    Plfydot_result_origin = Plfydot_result;      Plfzdot_result_origin = Plfzdot_result;
%       Left Hind (lh)
Plhxdot_result_origin = Plhxdot_result;    Plhydot_result_origin = Plhydot_result;      Plhzdot_result_origin = Plhzdot_result;
%       Right Front (rf)
Prfxdot_result_origin = Prfxdot_result;    Prfydot_result_origin = Prfydot_result;      Prfzdot_result_origin = Prfzdot_result;
%       Right Hind (rh)
Prhxdot_result_origin = Prhxdot_result;    Prhydot_result_origin = Prhydot_result;      Prhzdot_result_origin = Prhzdot_result;
%--------------------
%   Default Feet Locations in World Frame
%--------------------
%       Left Front (lf)
PlfxCenter_result_world_origin = PlfxCenter_result_world;     
PlfyCenter_result_world_origin = PlfyCenter_result_world;     
PlfzCenter_result_world_origin = PlfzCenter_result_world;
%       Left Hind (lh)
PlhxCenter_result_world_origin = PlhxCenter_result_world;     
PlhyCenter_result_world_origin = PlhyCenter_result_world;     
PlhzCenter_result_world_origin = PlhzCenter_result_world;
%       Right Front (rf)
PrfxCenter_result_world_origin = PrfxCenter_result_world;     
PrfyCenter_result_world_origin = PrfyCenter_result_world;     
PrfzCenter_result_world_origin = PrfzCenter_result_world;
%       Right Hind (rh)
PrhxCenter_result_world_origin = PrhxCenter_result_world;     
PrhyCenter_result_world_origin = PrhyCenter_result_world;     
PrhzCenter_result_world_origin = PrhzCenter_result_world;
%--------------------
%   Feet Forces
%--------------------
%       Left Front (lf)
Flfx_result_origin = Flfx_result;    Flfy_result_origin = Flfy_result;      Flfz_result_origin = Flfz_result;
%       Left Hind (lh)
Flhx_result_origin = Flhx_result;    Flhy_result_origin = Flhy_result;      Flhz_result_origin = Flhz_result;
%       Right Front (rf)
Frfx_result_origin = Frfx_result;    Frfy_result_origin = Frfy_result;      Frfz_result_origin = Frfz_result;
%       Right Hind (rh)
Frhx_result_origin = Frhx_result;    Frhy_result_origin = Frhy_result;      Frhz_result_origin = Frhz_result;
%       Net Forces
NetForceX_origin   = NetForceX;      NetForceY_origin   = NetForceY;        NetForceZ_origin   = NetForceZ;
%       Torques on the Torso
%       left Front(lf)
LeftFrontTorqueX_result_origin = LeftFrontTorqueX_result; 
LeftFrontTorqueY_result_origin = LeftFrontTorqueY_result;
LeftFrontTorqueZ_result_origin = LeftFrontTorqueZ_result; 
%       left Hind(lh)
LeftHindTorqueX_result_origin = LeftHindTorqueX_result;
LeftHindTorqueY_result_origin = LeftHindTorqueY_result;
LeftHindTorqueZ_result_origin = LeftHindTorqueZ_result;
%       right Front(rf)
RightFrontTorqueX_result_origin = RightFrontTorqueX_result;
RightFrontTorqueY_result_origin = RightFrontTorqueY_result;
RightFrontTorqueZ_result_origin = RightFrontTorqueZ_result;
%       right Hind(rf)
RightHindTorqueX_result_origin = RightHindTorqueX_result;
RightHindTorqueY_result_origin = RightHindTorqueY_result;
RightHindTorqueZ_result_origin = RightHindTorqueZ_result;

%       Net Torque
NetTorqueX_origin   = NetTorqueX;
NetTorqueY_origin   = NetTorqueY;
NetTorqueZ_origin   = NetTorqueZ;

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
%   Robot Torso State (Linear Position)
%---------------
x_result(find(TimeStepDiff <= 1e-3) + 1) = [];
y_result(find(TimeStepDiff <= 1e-3) + 1) = [];
z_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%---------------
%   Robot Torso State (Linear Velocity)
%---------------
xdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
ydot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
zdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%---------------
%   Robot Torso State (Orientation->Euler Angles)
%---------------
phi_result(find(TimeStepDiff <= 1e-3) + 1) = [];
theta_result(find(TimeStepDiff <= 1e-3) + 1) = [];
psi_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%---------------
%   Robot Torso State (Euler Angle Rates)
%---------------
phidot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
thetadot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
psidot_result(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Location
%---------------
%       Left Front (lf)
Plfx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plfy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plfz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Left Hind (lh)
Plhx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plhy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Plhz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Front (rf)
Prfx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prfy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prfz_result(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Hind (rh)
Prhx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prhy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
Prhz_result(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Velocity
%---------------
%       Left Front (lf)
Plfxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Plfydot_result(find(TimeStepDiff <= 1e-3)) = [];
Plfzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Left Hind (lh)
Plhxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Plhydot_result(find(TimeStepDiff <= 1e-3)) = [];
Plhzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Front (rf)
Prfxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Prfydot_result(find(TimeStepDiff <= 1e-3)) = [];
Prfzdot_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Hind (rh)
Prhxdot_result(find(TimeStepDiff <= 1e-3)) = [];
Prhydot_result(find(TimeStepDiff <= 1e-3)) = [];
Prhzdot_result(find(TimeStepDiff <= 1e-3)) = [];

%---------------
%   Default Feet Location
%---------------
%       Left Front (lf)
PlfxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlfyCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlfzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Left Hind (lh)
PlhxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlhyCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PlhzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Front (rf)
PrfxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrfyCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrfzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
%       Right Hind (rh)
PrhxCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrhyCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
PrhzCenter_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

%---------------
%   Feet Forces
%---------------
%       Left Front (lf)
Flfx_result(find(TimeStepDiff <= 1e-3)) = [];
Flfy_result(find(TimeStepDiff <= 1e-3)) = [];
Flfz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Left Hind (lh)
Flhx_result(find(TimeStepDiff <= 1e-3)) = [];
Flhy_result(find(TimeStepDiff <= 1e-3)) = [];
Flhz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Front (rf)
Frfx_result(find(TimeStepDiff <= 1e-3)) = [];
Frfy_result(find(TimeStepDiff <= 1e-3)) = [];
Frfz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Right Hind (rh)
Frhx_result(find(TimeStepDiff <= 1e-3)) = [];
Frhy_result(find(TimeStepDiff <= 1e-3)) = [];
Frhz_result(find(TimeStepDiff <= 1e-3)) = [];
%       Net Forces
NetForceX(find(TimeStepDiff <= 1e-3)) = [];
NetForceY(find(TimeStepDiff <= 1e-3)) = [];
NetForceZ(find(TimeStepDiff <= 1e-3)) = [];
%       Net Torque
NetTorqueX(find(TimeStepDiff <= 1e-3)) = [];
NetTorqueY(find(TimeStepDiff <= 1e-3)) = [];
NetTorqueZ(find(TimeStepDiff <= 1e-3)) = [];

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

%*Sequence is consistent with the programming Clf -> Clh -> Crf -> Crh*
gait = [Clf_result,...
        Clh_result,...
        Crf_result,...
        Crh_result,...
        PhaseLengths];

warning('NOTE 1: For display, Gait Matrix Sequence Becomes "gait_display": LH -> LF -> RF -> RH')
warning('NOTE 2: The Gait Matric for display is the Transpose of the original column vectors (Each LINE Represnet the Contact Sequence of a Limb)')
warning('NOTE 3: "gait" is the SOLELY variable name storing the gait sequence which is consistent with respect to the optimization program, and it is Read COLUMN by COLUMN')
gait_display = [Clh_result,...
                Clf_result,...
                Crf_result,...
                Crh_result,...
                PhaseLengths];

gait_display'

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
