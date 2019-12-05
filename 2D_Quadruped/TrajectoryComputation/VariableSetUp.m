% Script for Setting Up Varibale for 2D quadruped

%   Limb Naming (From Top View)
%    lf------------rf
%     |            |
%     |            |
%     |            |
%     |            |
%     |            |
%     |            |
%     |            |
%    rf------------rh

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
% x: Horizontal Position of the torso
x = SX.sym('x', tauSeriesLength); 

% z: Vertical Position of the torso
z = SX.sym('z', tauSeriesLength); 

% theta: Torso Orientation
theta = SX.sym('theta', tauSeriesLength);

% xdot: Horizontal velocity of the torso
xdot = SX.sym('xdot', tauSeriesLength); 

% zdot: Vertical velocity of the torso
zdot = SX.sym('zdot', tauSeriesLength);

%thetadot: angular velocity of the torso
thetadot = SX.sym('thetadot', tauSeriesLength);

%--------------------------------------------------------------------------
%   Limb Variables: Feet Locations in [World Frame]
%--------------------------------------------------------------------------
% Left Front (lf) Feet Position Vector -> Plf = [Plfx, Plfz]:
Plfx = SX.sym('Plfx',tauSeriesLength);
Plfz = SX.sym('Plfz',tauSeriesLength);

% Left Hind (lh) Feet Position Vector -> Plh = [Plhx, Plhz]:
Plhx = SX.sym('Plhx',tauSeriesLength);
Plhz = SX.sym('Plhz',tauSeriesLength);

% Right Front (rf) Feet Position Vector -> Prf = [Prfx, Prfz]:
Prfx = SX.sym('Prfx',tauSeriesLength);
Prfz = SX.sym('Prfz',tauSeriesLength);

% Right Hind (rh) Feet Position Vector -> Prh = [Prhx, Prhz]:
Prhx = SX.sym('Prhx',tauSeriesLength);
Prhz = SX.sym('Prhz',tauSeriesLength);

%--------------------------------------------------------------------------
%   Limb Variables: Feet Velocities in [World Frame]
%       NOTE: As inputs to the system and we use Euler integration, the
%       length of Feet Velocity vector is "tauSeriesLength - 1"
%--------------------------------------------------------------------------
% Left Front (lf) Feet Velocity Vector -> Plfdot = [Plfxdot, Plfzdot]:
Plfxdot = SX.sym('Plfxdot',tauSeriesLength - 1);
Plfzdot = SX.sym('Plfzdot',tauSeriesLength - 1);

% Left Hind (lh) Feet Position Vector -> Plh = [Plhx, Plhz]:
Plhxdot = SX.sym('Plhxdot',tauSeriesLength - 1);
Plhzdot = SX.sym('Plhzdot',tauSeriesLength - 1);

% Right Front (rf) Feet Position Vector -> Prf = [Prfx, Prfz]:
Prfxdot = SX.sym('Prfxdot',tauSeriesLength - 1);
Prfzdot = SX.sym('Prfzdot',tauSeriesLength - 1);

% Right Hind (rh) Feet Position Vector -> Prh = [Prhx, Prhz]:
Prhxdot = SX.sym('Prhxdot',tauSeriesLength - 1);
Prhzdot = SX.sym('Prhzdot',tauSeriesLength - 1);

%--------------------------------------------------------------------------
%   Limb Variables: Foot-Ground Reaction Forces in [World Frame]
%       NOTE: As inputs to the system and we use Euler integration, the
%       length of Foot-Ground Reaction Forces vector is "tauSeriesLength - 1"
%--------------------------------------------------------------------------
% Left Front (lf) Foot-Ground Reaction Forces Vector -> Flf = [Flfx, Flfz]:
Flfx = SX.sym('Flfx',tauSeriesLength - 1);
Flfz = SX.sym('Flfz',tauSeriesLength - 1);

% Left Hind (lh) Foot-Ground Reaction Forces Vector -> Flh = [Flhx, Flhz]:
Flhx = SX.sym('Flhx',tauSeriesLength - 1);
Flhz = SX.sym('Flhz',tauSeriesLength - 1);

% Right Front (rf) Foot-Ground Reaction Forces Vector -> Frf = [Frfx, Frfz]:
Frfx = SX.sym('Frfx',tauSeriesLength - 1);
Frfz = SX.sym('Frfz',tauSeriesLength - 1);

% Right Hind (rh) Foot-Ground Reaction Forces Vector -> Frh = [Frhx, Frhz]:
Frhx = SX.sym('Frhx',tauSeriesLength - 1);
Frhz = SX.sym('Frhz',tauSeriesLength - 1);



