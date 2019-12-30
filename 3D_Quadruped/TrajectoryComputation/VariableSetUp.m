% Script for Setting Up Varibale for 3D quadruped

%   Limb Naming (From Top View)
%    lf(1)------------rf(3)
%        |            |
%        |            |
%        |            |
%        |            |
%        |            |
%        |            |
%        |            |
%    lh(2)------------rh(4)

% Import CasADi related packages
import casadi.*

%==========================================================================
% Define/Create Variable for the 2D quadruped model
%==========================================================================
%   Display Info
disp('====================================================');
disp('Generate Decision Variables Vectors/Lists:')
disp('----------------------------------------------------');
%--------------------------------------------------------------------------
%   Torso Variables [In World Frame]
%--------------------------------------------------------------------------

%   Linear Positions

% x: Horizontal Position of the torso --> x-axis (In World Frame)
x = SX.sym('x', tauSeriesLength); 

% y: Horizontal Position of the torso --> y-axis (In World Frame)
y = SX.sym('y', tauSeriesLength); 

% z: Vertical Position of the torso
z = SX.sym('z', tauSeriesLength); 

%   Linear Velocities

% xdot: Horizontal velocity of the torso --> x-axis (In World Frame)
xdot = SX.sym('xdot', tauSeriesLength); 

% ydot: Horizontal velocity of the torso --> y-axis (In World Frame)
ydot = SX.sym('ydot', tauSeriesLength); 

% zdot: Vertical velocity of the torso
zdot = SX.sym('zdot', tauSeriesLength);

%   Orientation using Euler Angles --> ZYX: yaw(psi) - pitch(theta) - roll(phi)
%       The sequence is important for the application sequence of
%       elementary rotations, but not important when we place them in a
%       vector

% phi: roll
phi   = SX.sym('phi', tauSeriesLength);

% theta: pitch
theta = SX.sym('theta', tauSeriesLength);

% psi: yaw
psi   = SX.sym('psi', tauSeriesLength);

%   Euler Angle Rates

% phidot: roll dot
phidot   = SX.sym('phidot', tauSeriesLength);

% thetadot: pitch dot
thetadot = SX.sym('thetadot', tauSeriesLength);

% yawdot: yaw dot
psidot   = SX.sym('psidot', tauSeriesLength);

%--------------------------------------------------------------------------
%   Limb Variables: Feet Locations in [World Frame]
%--------------------------------------------------------------------------
% Left Front (lf) Feet Position Vector -> Plf = [Plfx, Plfy, Plfz]:
Plfx = SX.sym('Plfx',tauSeriesLength);
Plfy = SX.sym('Plfy',tauSeriesLength);
Plfz = SX.sym('Plfz',tauSeriesLength);

% Left Hind (lh) Feet Position Vector -> Plh = [Plhx, Plhy, Plhz]:
Plhx = SX.sym('Plhx',tauSeriesLength);
Plhy = SX.sym('Plhy',tauSeriesLength);
Plhz = SX.sym('Plhz',tauSeriesLength);

% Right Front (rf) Feet Position Vector -> Prf = [Prfx, Prfy, Prfz]:
Prfx = SX.sym('Prfx',tauSeriesLength);
Prfy = SX.sym('Prfy',tauSeriesLength);
Prfz = SX.sym('Prfz',tauSeriesLength);

% Right Hind (rh) Feet Position Vector -> Prh = [Prhx, Prhy, Prhz]:
Prhx = SX.sym('Prhx',tauSeriesLength);
Prhy = SX.sym('Prhy',tauSeriesLength);
Prhz = SX.sym('Prhz',tauSeriesLength);

%--------------------------------------------------------------------------
%   Limb Variables: Feet Velocities in [World Frame]
%       NOTE: As inputs to the system and we use Euler integration, the
%       length of Feet Velocity vector is "tauSeriesLength - 1"
%--------------------------------------------------------------------------
% Left Front (lf) Feet Velocity Vector -> Plfdot = [Plfxdot, Plfydot, Plfzdot]:
Plfxdot = SX.sym('Plfxdot',tauSeriesLength - 1);
Plfydot = SX.sym('Plfydot',tauSeriesLength - 1);
Plfzdot = SX.sym('Plfzdot',tauSeriesLength - 1);

% Left Hind (lh) Feet Position Vector -> Plhdot = [Plhxdot, Plhydot, Plhzdot]:
Plhxdot = SX.sym('Plhxdot',tauSeriesLength - 1);
Plhydot = SX.sym('Plhydot',tauSeriesLength - 1);
Plhzdot = SX.sym('Plhzdot',tauSeriesLength - 1);

% Right Front (rf) Feet Position Vector -> Prfdot = [Prfxdot, Prfydot, Prfzdot]:
Prfxdot = SX.sym('Prfxdot',tauSeriesLength - 1);
Prfydot = SX.sym('Prfydot',tauSeriesLength - 1);
Prfzdot = SX.sym('Prfzdot',tauSeriesLength - 1);

% Right Hind (rh) Feet Position Vector -> Prhdot = [Prhxdot, Prhydot, Prhzdot]:
Prhxdot = SX.sym('Prhxdot',tauSeriesLength - 1);
Prhydot = SX.sym('Prhydot',tauSeriesLength - 1);
Prhzdot = SX.sym('Prhzdot',tauSeriesLength - 1);

%--------------------------------------------------------------------------
%   Limb Variables: Foot-Ground Reaction Forces in [World Frame]
%       NOTE: As inputs to the system and we use Euler integration, the
%       length of Foot-Ground Reaction Forces vector is "tauSeriesLength - 1"
%--------------------------------------------------------------------------
% Left Front (lf) Foot-Ground Reaction Forces Vector -> Flf = [Flfx, Flfy, Flfz]:
Flfx = SX.sym('Flfx',tauSeriesLength - 1);
Flfy = SX.sym('Flfy',tauSeriesLength - 1);
Flfz = SX.sym('Flfz',tauSeriesLength - 1);

% Left Hind (lh) Foot-Ground Reaction Forces Vector -> Flh = [Flhx, Flhy, Flhz]:
Flhx = SX.sym('Flhx',tauSeriesLength - 1);
Flhy = SX.sym('Flhy',tauSeriesLength - 1);
Flhz = SX.sym('Flhz',tauSeriesLength - 1);

% Right Front (rf) Foot-Ground Reaction Forces Vector -> Frf = [Frfx, Frfy, Frfz]:
Frfx = SX.sym('Frfx',tauSeriesLength - 1);
Frfy = SX.sym('Frfy',tauSeriesLength - 1);
Frfz = SX.sym('Frfz',tauSeriesLength - 1);

% Right Hind (rh) Foot-Ground Reaction Forces Vector -> Frh = [Frhx, Frhy, Frhz]:
Frhx = SX.sym('Frhx',tauSeriesLength - 1);
Frhy = SX.sym('Frhy',tauSeriesLength - 1);
Frhz = SX.sym('Frhz',tauSeriesLength - 1);

%--------------------------------------------------------------------------
%   Switching Time for Each Phase (Phase Duration)
%       Represent the termination time of each phase
%
%  |__________|_________|_________|________|
%  0         Ts1        Ts2      Ts3      Ts4(Tend)
%--------------------------------------------------------------------------
Ts = SX.sym('Ts', NumPhases + 1); %Make variables from Ts_0 to Ts_NumPhases
% Remove Ts0 which is fixed to be 0
Ts = Ts(2:end);

%--------------------------------------------------------------------------
%   Contact Configurations
%  |__________|_________|_________|________|
%  0         Ts1        Ts2      Ts3      Ts4(Tend)
%       C1         C2        C3       C4
%--------------------------------------------------------------------------
% Build Contact Configuration vector only if we want to perform free gait
% discovery
if gait_discovery_switch == 1 %yes, we want free gait discovery
    
    % Left Front (lf)
    Clf = SX.sym('Clf', NumPhases + 1); %from C_0 to C_NumPhases
    Clf = Clf(2:end); %remove C_0
    
    %Left Hind (lh)
    Clh = SX.sym('Clh', NumPhases + 1); %from C_0 to C_NumPhases
    Clh = Clh(2:end); %remove C_0
    
    % Right Front (rf)
    Crf = SX.sym('Crf', NumPhases + 1); %from C_0 to C_NumPhases
    Crf = Crf(2:end); %remove C_0

    % Right Hind (rh)
    Crh = SX.sym('Crh', NumPhases + 1); %from C_0 to C_NumPhases
    Crh = Crh(2:end); %remove C_0
end


%==========================================================================
% Create Name Lists for Variables
%==========================================================================
%--------------------------------------------------------------------------
% Name List for individual quantities
%--------------------------------------------------------------------------
% Torso Variabls
%   Linear Positions
x_label        = CreateVarsNameList(x);
y_label        = CreateVarsNameList(y);
z_label        = CreateVarsNameList(z);
%   Linear Velocities
xdot_label     = CreateVarsNameList(xdot);
ydot_label     = CreateVarsNameList(ydot);
zdot_label     = CreateVarsNameList(zdot);
%   Orientation
phi_label      = CreateVarsNameList(phi); %roll
theta_label    = CreateVarsNameList(theta); %pitch
psi_label      = CreateVarsNameList(psi); %yaw
%   Euler Angle rate
phidot_label   = CreateVarsNameList(phidot); %roll dot
thetadot_label = CreateVarsNameList(thetadot); %pitch dot
psidot_label   = CreateVarsNameList(psidot); %yaw dot

% Limb Variables (Feet Locations)
%   Left Front (lf)
Plfx_label   = CreateVarsNameList(Plfx);
Plfy_label   = CreateVarsNameList(Plfy);    
Plfz_label   = CreateVarsNameList(Plfz);
%   Left Hind (lh)
Plhx_label   = CreateVarsNameList(Plhx);   
Plhy_label   = CreateVarsNameList(Plhy);  
Plhz_label   = CreateVarsNameList(Plhz); 
%   Right Front (rf)
Prfx_label   = CreateVarsNameList(Prfx); 
Prfy_label   = CreateVarsNameList(Prfy);   
Prfz_label   = CreateVarsNameList(Prfz); 
%   Right Hind (rh)
Prhx_label   = CreateVarsNameList(Prhx);   
Prhy_label   = CreateVarsNameList(Prhy);
Prhz_label   = CreateVarsNameList(Prhz);

% Limb Variables (Feet Velocities)
%   Left Front (lf)
Plfxdot_label = CreateVarsNameList(Plfxdot); 
Plfydot_label = CreateVarsNameList(Plfydot);
Plfzdot_label = CreateVarsNameList(Plfzdot); 
%   Left Hind (lh)
Plhxdot_label = CreateVarsNameList(Plhxdot); 
Plhydot_label = CreateVarsNameList(Plhydot);
Plhzdot_label = CreateVarsNameList(Plhzdot); 
%   Right Front (rf)
Prfxdot_label = CreateVarsNameList(Prfxdot); 
Prfydot_label = CreateVarsNameList(Prfydot); 
Prfzdot_label = CreateVarsNameList(Prfzdot); 
%   Right Hind (rh)
Prhxdot_label = CreateVarsNameList(Prhxdot);
Prhydot_label = CreateVarsNameList(Prhydot);
Prhzdot_label = CreateVarsNameList(Prhzdot); 

% Limb Variables (Feet-Ground Reaction Forces)
%   Left Front (lf)
Flfx_label  = CreateVarsNameList(Flfx);  
Flfy_label  = CreateVarsNameList(Flfy);  
Flfz_label  = CreateVarsNameList(Flfz); 
%   Left Hind (lh)
Flhx_label  = CreateVarsNameList(Flhx);   
Flhy_label  = CreateVarsNameList(Flhy); 
Flhz_label  = CreateVarsNameList(Flhz); 
%   Right Front (rf)
Frfx_label  = CreateVarsNameList(Frfx);     
Frfy_label  = CreateVarsNameList(Frfy); 
Frfz_label  = CreateVarsNameList(Frfz); 
%   Right Hind (rh)
Frhx_label  = CreateVarsNameList(Frhx); 
Frhy_label  = CreateVarsNameList(Frhy);
Frhz_label  = CreateVarsNameList(Frhz); 

% Switching Time (Phase Duration)
Ts_label    = CreateVarsNameList(Ts);

% Contact Configuration
if gait_discovery_switch == 1 %yes, we want free gait discovery then we build Contact Configuration List
    Clf_label   = CreateVarsNameList(Clf); %Left Front (lf)
    Clh_label   = CreateVarsNameList(Clh); %Left Hind (lh)
    Crf_label   = CreateVarsNameList(Crf); %Right Front (rf)
    Crh_label   = CreateVarsNameList(Crh); %Right Hind (rh)
end

%--------------------------------------------------------------------------
% Full Name/Length List containing all variables
%--------------------------------------------------------------------------
% Common Part
%-------------
%   varCategoryList -> list quantities and their sequence placed in the
%   decision varibale vector
VarCategoryList = ["x",         "y",         "z",... Linear Position
                   "xdot",      "ydot",      "zdot",... Linear Velocity
                   "phi",       "theta",     "psi",...Orientation
                   "phidot",    "thetadot",  "psidot",...Orientation Rate
                   "Plfx",      "Plfy",      "Plfz",...Left Front (lf) Feet Location
                   "Plhx",      "Plhy",      "Plhz",...Left Hind (lh) Feet Location
                   "Prfx",      "Prfy",      "Prfz",...Right Front (rf) Feet Location
                   "Prhx",      "Prhy",      "Prhy",...Right Hind (rh) Feet Location
                   "Plfxdot",   "Plfydot",   "Plfzdot",...Left Front (lf) Feet Velocity
                   "Plhxdot",   "Plhydot",   "Plhzdot"...Left Hind (lh) Feet Velocity
                   "Prfxdot",   "Prfydot",   "Prfzdot",...Right Front (rf) Feet Velocity
                   "Prhxdot",   "Prhydot",   "Prhzdot",...Right Hind (rh) Feet Velocity
                   "Flfx",      "Flfy",      "Flfz",...Left Front (lf) Feet Forces
                   "Flhx",      "Flhy",      "Flhz",...Left Hind (lh) Feet Forces
                   "Frfx",      "Frfy",      "Frfz",...Right Front (rf) Feet Forces
                   "Frhx",      "Frhy",      "Frhz",...Right Hind (rh) Feet Forces
                   "Ts"];
%-------------
%   varLengthList -> List of lengths of each quantity
VarLengthList = [length(x_label),       length(z_label),        length(theta_label),...
                 length(xdot_label),    length(zdot_label),     length(thetadot_label),...
                 length(Plfx_label),    length(Plfz_label),...
                 length(Plhx_label),    length(Plhz_label),...
                 length(Prfx_label),    length(Prfz_label),...
                 length(Prhx_label),    length(Prhz_label),...
                 length(Plfxdot_label), length(Plfzdot_label),...
                 length(Plhxdot_label), length(Plhzdot_label),...
                 length(Prfxdot_label), length(Prfzdot_label),...
                 length(Prhxdot_label), length(Prhzdot_label),...
                 length(Flfx_label),    length(Flfz_label),...
                 length(Flhx_label),    length(Flhz_label),...
                 length(Frfx_label),    length(Frfz_label),...
                 length(Frhx_label),    length(Frhz_label),...
                 length(Ts_label)];
%-------------
%   Full Deicision Variable Name List
VarNamesList = [x_label,        z_label,        theta_label,...
                xdot_label,     zdot_label,     thetadot_label,...
                Plfx_label,     Plfz_label,...
                Plhx_label,     Plhz_label,...
                Prfx_label,     Prfz_label,...
                Prhx_label,     Prhz_label,...
                Plfxdot_label,  Plfzdot_label,...
                Plhxdot_label,  Plhzdot_label,...
                Prfxdot_label,  Prfzdot_label,...
                Prhxdot_label,  Prhzdot_label,...
                Flfx_label,     Flfz_label,...
                Flhx_label,     Flhz_label,...
                Frfx_label,     Frfz_label,...
                Frhx_label,     Frhz_label,...
                Ts_label];
%-------------
% Extend the list with contact configurations if we perform gait discovery
%-------------
if gait_discovery_switch == 1 %Yes, we want free gait discovery
    
    VarCategoryList = [VarCategoryList,...
                       "Clf",...
                       "Clh",...
                       "Crf",...
                       "Crh"];
                   
    VarLengthList = [VarLengthList,...
                     length(Clf_label),...
                     length(Clh_label),...
                     length(Crf_label),...
                     length(Crh_label)];
                 
    VarNamesList = [VarNamesList,...
                    Clf_label,...
                    Clh_label,...
                    Crf_label,...
                    Crh_label];
                
end

%--------------------------------------------------------------------------
% Build Decision Variables List for Optimization
%--------------------------------------------------------------------------
% Collect copntinuous variables -> For Fixed Gait Computation
DecisionVars = {x,        z,        theta,...
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
               Ts};
% Add contact configurations into the list if we want gait discovery
if gait_discovery_switch == 1 %Yes, we want free gait discovery
    DecisionVars{end+1} = Clf;
    DecisionVars{end+1} = Clh;
    DecisionVars{end+1} = Crf;
    DecisionVars{end+1} = Crh;
end

DecisionVars = vertcat(DecisionVars{:}); %make a vertical vector

%--------------------------------------------------------------------------
% Build Variable Type List
%--------------------------------------------------------------------------
%  0 -> Continuous Variable/ 1 -> Binary Variable
varstype = contains(VarNamesList, 'C');

%--------------------------------------------------------------------------
% Verification Step
%--------------------------------------------------------------------------
if sum(VarNamesList == CreateVarsNameList(DecisionVars)) == sum(VarLengthList)
    disp('Decision Variable List Built Successfully')
else
    error('Decision Variable List does not match with Variable Name List');
end
disp('====================================================');
disp(' ')

