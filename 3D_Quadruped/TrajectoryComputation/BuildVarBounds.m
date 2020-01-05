% Script for Building Variable Bounds

%   Build lower and upper bounds --> Follow the order Defined in
%   VarCategoryList
%   VarCategoryList = ["x",         "y",         "z",... Linear Position
%                      "xdot",      "ydot",      "zdot",... Linear Velocity
%                      "phi",       "theta",     "psi",...Orientation
%                      "phidot",    "thetadot",  "psidot",...Orientation Rate
%                      "Plfx",      "Plfy",      "Plfz",...Left Front (lf) Feet Location
%                      "Plhx",      "Plhy",      "Plhz",...Left Hind (lh) Feet Location
%                      "Prfx",      "Prfy",      "Prfz",...Right Front (rf) Feet Location
%                      "Prhx",      "Prhy",      "Prhy",...Right Hind (rh) Feet Location
%                      "Plfxdot",   "Plfydot",   "Plfzdot",...Left Front (lf) Feet Velocity
%                      "Plhxdot",   "Plhydot",   "Plhzdot"...Left Hind (lh) Feet Velocity
%                      "Prfxdot",   "Prfydot",   "Prfzdot",...Right Front (rf) Feet Velocity
%                      "Prhxdot",   "Prhydot",   "Prhzdot",...Right Hind (rh) Feet Velocity
%                      "Flfx",      "Flfy",      "Flfz",...Left Front (lf) Feet Forces
%                      "Flhx",      "Flhy",      "Flhz",...Left Hind (lh) Feet Forces
%                      "Frfx",      "Frfy",      "Frfz",...Right Front (rf) Feet Forces
%                      "Frhx",      "Frhy",      "Frhz",...Right Hind (rh) Feet Forces
%                      "Ts",...
%                      "Clf",...
%                      "Clh",...
%                      "Crf",...
%                      "Crh"];

%       x-axis
lb_x = repmat(-0.1, 1, VarLengthList(VarCategoryList == "x"));
if SpeedDirection == 1 %Horizontal Speed
    ub_x = repmat( 1.5*speed*Tend, 1, VarLengthList(VarCategoryList == "x"));
elseif SpeedDirection == 2 %Tangential Speed
    ub_x = repmat( 1.5*speed*Tend*cos(terrain_slope_rad), 1, VarLengthList(VarCategoryList == "x"));
end

%       y-axis
lb_y = repmat(-5, 1, VarLengthList(VarCategoryList == "y"));    
ub_y = repmat( 5, 1, VarLengthList(VarCategoryList == "y"));

%       z-axis            
if TerrainType == 1 %Flat Terrain
    lb_z = repmat(-5, 1, VarLengthList(VarCategoryList == "z"));
    ub_z = repmat( 5, 1, VarLengthList(VarCategoryList == "z"));
elseif TerrainType == 2 %slope
    if SpeedDirection == 1 %Horizontal Desired Speed
        lb_z = repmat(-5 - 2*abs(speed*Tend*tan(terrain_slope_rad)), 1, VarLengthList(VarCategoryList == "z"));  
        ub_z = repmat( 5 + 2*abs(speed*Tend*tan(terrain_slope_rad)), 1, VarLengthList(VarCategoryList == "z"));
    elseif SpeedDirection == 2 %Tangential Desired Speed
        lb_z = repmat(-5 - 2*abs(speed*Tend*sin(terrain_slope_rad)), 1, VarLengthList(VarCategoryList == "z"));    
        ub_z = repmat( 5 + 2*abs(speed*Tend*sin(terrain_slope_rad)), 1, VarLengthList(VarCategoryList == "z"));
    end
end

%       xdot
lb_xdot = repmat(-5*speed, 1, VarLengthList(VarCategoryList == "xdot"));    
ub_xdot = repmat( 5*speed, 1, VarLengthList(VarCategoryList == "xdot"));

%       ydot
lb_ydot = repmat(-25, 1, VarLengthList(VarCategoryList == "ydot"));    
ub_ydot = repmat( 25, 1, VarLengthList(VarCategoryList == "ydot"));

%       zdot
lb_zdot = repmat(-25, 1, VarLengthList(VarCategoryList == "zdot"));    
ub_zdot = repmat( 25, 1, VarLengthList(VarCategoryList == "zdot"));

%       phi
lb_phi = repmat(-pi/2, 1, VarLengthList(VarCategoryList == "phi"));    
ub_phi = repmat( pi/2, 1, VarLengthList(VarCategoryList == "phi"));

%       theta
lb_theta = repmat(-pi/2,          1, VarLengthList(VarCategoryList == "theta"));    
ub_theta = repmat( 90/100*(pi/2), 1, VarLengthList(VarCategoryList == "theta")); %90% of pi/2

%       psi
lb_psi = repmat(-pi/2, 1, VarLengthList(VarCategoryList == "psi"));    
ub_psi = repmat( pi/2, 1, VarLengthList(VarCategoryList == "psi")); 

%       phi dot
lb_phidot = repmat(-5*pi, 1, VarLengthList(VarCategoryList == "phidot"));    
ub_phidot = repmat( 5*pi, 1, VarLengthList(VarCategoryList == "phidot"));

%       theta dot
lb_thetadot = repmat(-5*pi, 1, VarLengthList(VarCategoryList == "thetadot"));    
ub_thetadot = repmat( 5*pi, 1, VarLengthList(VarCategoryList == "thetadot"));

%       psi dot
lb_psidot = repmat(-5*pi, 1, VarLengthList(VarCategoryList == "psidot"));    
ub_psidot = repmat( 5*pi, 1, VarLengthList(VarCategoryList == "psidot"));

%       Plfx
lb_Plfx = lb_x + repmat(-5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Plfx"));  
ub_Plfx = ub_x + repmat( 5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Plfx"));
%       Plfy
lb_Plfy = lb_y + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plfy"));  
ub_Plfy = ub_y + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plfy"));
%       Plfz
lb_Plfz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plfz"));  
ub_Plfz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plfz"));

%       Plhx
lb_Plhx = lb_x + repmat(-5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Plhx"));  
ub_Plhx = ub_x + repmat( 5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Plhx"));
%       Plhy
lb_Plhy = lb_y + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plhy"));  
ub_Plhy = ub_y + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plhy"));
%       Plhz
lb_Plhz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plhz"));  
ub_Plhz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plhz"));

%       Prfx
lb_Prfx = lb_x + repmat(-5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Prfx"));  
ub_Prfx = ub_x + repmat( 5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Prfx"));
%       Prfx
lb_Prfy = lb_y + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prfy"));  
ub_Prfy = ub_y + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prfy"));
%       Prfz
lb_Prfz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prfz"));  
ub_Prfz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prfz"));

%       Prhx
lb_Prhx = lb_x + repmat(-5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Prhx"));  
ub_Prhx = ub_x + repmat( 5*BoundingBox_Length, 1, VarLengthList(VarCategoryList == "Prhx"));
%       Prhy
lb_Prhy = lb_y + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prhy"));  
ub_Prhy = ub_y + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prhy"));
%       Prhz
lb_Prhz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prhz"));  
ub_Prhz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prhz"));

%       Plfxdot
lb_Plfxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plfxdot"));
ub_Plfxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plfxdot"));
%       Plfydot
lb_Plfydot = lb_ydot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plfydot"));
ub_Plfydot = ub_ydot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plfydot"));
%       Plfzdot
lb_Plfzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(3), 1, VarLengthList(VarCategoryList == "Plfzdot"));
ub_Plfzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(3), 1, VarLengthList(VarCategoryList == "Plfzdot"));

%       Plhxdot
lb_Plhxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plhxdot"));
ub_Plhxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plhxdot"));
%       Plhydot
lb_Plhydot = lb_ydot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plhydot"));
ub_Plhydot = ub_ydot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plhydot"));
%       Plhzdot
lb_Plhzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(3), 1, VarLengthList(VarCategoryList == "Plhzdot"));
ub_Plhzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(3), 1, VarLengthList(VarCategoryList == "Plhzdot"));

%       Prfxdot
lb_Prfxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prfxdot"));
ub_Prfxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prfxdot"));
%       Prfydot
lb_Prfydot = lb_ydot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prfydot"));
ub_Prfydot = ub_ydot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prfydot"));
%       Prfzdot
lb_Prfzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(3), 1, VarLengthList(VarCategoryList == "Prfzdot"));
ub_Prfzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(3), 1, VarLengthList(VarCategoryList == "Prfzdot"));

%       Prhxdot
lb_Prhxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prhxdot"));
ub_Prhxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prhxdot"));
%       Prhydot
lb_Prhydot = lb_ydot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prhydot"));
ub_Prhydot = ub_ydot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prhydot"));
%       Prhzdot
lb_Prhzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(3), 1, VarLengthList(VarCategoryList == "Prhzdot"));
ub_Prhzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(3), 1, VarLengthList(VarCategoryList == "Prhzdot"));

%       Flfx
lb_Flfx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flfx"));    
ub_Flfx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flfx"));
%       Flfy
lb_Flfy = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flfy"));    
ub_Flfy = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flfy"));
%       Flfz
lb_Flfz = repmat(-1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Flfz"));    
ub_Flfz = repmat( 1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Flfz"));

%       Flhx
lb_Flhx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flhx"));    
ub_Flhx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flhx"));
%       Flhy
lb_Flhy = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flhy"));    
ub_Flhy = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flhy"));
%       Flhz
lb_Flhz = repmat(-1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Flhz"));    
ub_Flhz = repmat( 1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Flhz"));

%       Frfx
lb_Frfx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frfx"));    
ub_Frfx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frfx"));
%       Frfy
lb_Frfy = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frfy"));    
ub_Frfy = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frfy"));
%       Frfz
lb_Frfz = repmat(-1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Frfz"));    
ub_Frfz = repmat( 1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Frfz"));

%       Frhx
lb_Frhx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frhx"));    
ub_Frhx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frhx"));
%       Frhy
lb_Frhy = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frhy"));    
ub_Frhy = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frhy"));
%       Frhz
lb_Frhz = repmat(-1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Frhz"));    
ub_Frhz = repmat( 1.5*Mf(3), 1, VarLengthList(VarCategoryList == "Frhz"));

%       Ts
lb_Ts = repmat( 0,             1, VarLengthList(VarCategoryList == "Ts"));  
ub_Ts = repmat( Tend+0.1*Tend, 1, VarLengthList(VarCategoryList == "Ts"));

if gait_discovery_switch == 1 % Discover Gait via MINLP
    %       Clf
    lb_Clf = zeros(1,  VarLengthList(VarCategoryList == "Clf"));    
    ub_Clf = ones( 1,  VarLengthList(VarCategoryList == "Clf"));
    %       Clh
    lb_Clh = zeros(1,  VarLengthList(VarCategoryList == "Clh"));    
    ub_Clh = ones( 1,  VarLengthList(VarCategoryList == "Clh"));
    %       Crf
    lb_Crf = zeros(1,  VarLengthList(VarCategoryList == "Crf"));    
    ub_Crf = ones( 1,  VarLengthList(VarCategoryList == "Crf"));
    %       Crh
    lb_Crh = zeros(1,  VarLengthList(VarCategoryList == "Crh"));    
    ub_Crh = ones( 1,  VarLengthList(VarCategoryList == "Crh"));
end

% Construct Final Big Vector for Variable Bounds
%   Build lb.ub for Continuous Variables
lb_ContinuousDecisionVars =     [lb_x,         lb_y,         lb_z,... Linear Position
                                 lb_xdot,      lb_ydot,      lb_zdot,... Linear Velocity
                                 lb_phi,       lb_theta,     lb_psi,...Orientation
                                 lb_phidot,    lb_thetadot,  lb_psidot,...Orientation Rate
                                 lb_Plfx,      lb_Plfy,      lb_Plfz,...Left Front (lf) Feet Location
                                 lb_Plhx,      lb_Plhy,      lb_Plhz,...Left Hind (lh) Feet Location
                                 lb_Prfx,      lb_Prfy,      lb_Prfz,...Right Front (rf) Feet Location
                                 lb_Prhx,      lb_Prhy,      lb_Prhz,...Right Hind (rh) Feet Velocity
                                 lb_Plfxdot,   lb_Plfydot,   lb_Plfzdot,...Left Front (lf) Feet Velocity
                                 lb_Plhxdot,   lb_Plhydot,   lb_Plhzdot,...Left Hind (lh) Feet Velocity
                                 lb_Prfxdot,   lb_Prfydot,   lb_Prfzdot,...Right Front (rf) Feet Velocity
                                 lb_Prhxdot,   lb_Prhydot,   lb_Prhzdot,...Right Hind (rh) Feet Velocity
                                 lb_Flfx,      lb_Flfy,      lb_Flfz,...Left Front (lf) Feet Forces
                                 lb_Flhx,      lb_Flhy,      lb_Flhz,...Left Hind (lh) Feet Forces
                                 lb_Frfx,      lb_Frfy,      lb_Frfz,...Right Front (rf) Feet Forces
                                 lb_Frhx,      lb_Frhy,      lb_Frhz,...Right Hind (rh) Feet Forces
                                 lb_Ts];
               
ub_ContinuousDecisionVars =     [ub_x,         ub_y,         ub_z,... Linear Position
                                 ub_xdot,      ub_ydot,      ub_zdot,... Linear Velocity
                                 ub_phi,       ub_theta,     ub_psi,...Orientation
                                 ub_phidot,    ub_thetadot,  ub_psidot,...Orientation Rate
                                 ub_Plfx,      ub_Plfy,      ub_Plfz,...Left Front (lf) Feet Location
                                 ub_Plhx,      ub_Plhy,      ub_Plhz,...Left Hind (lh) Feet Location
                                 ub_Prfx,      ub_Prfy,      ub_Prfz,...Right Front (rf) Feet Location
                                 ub_Prhx,      ub_Prhy,      ub_Prhz,...Right Hind (rh) Feet Velocity
                                 ub_Plfxdot,   ub_Plfydot,   ub_Plfzdot,...Left Front (lf) Feet Velocity
                                 ub_Plhxdot,   ub_Plhydot,   ub_Plhzdot,...Left Hind (lh) Feet Velocity
                                 ub_Prfxdot,   ub_Prfydot,   ub_Prfzdot,...Right Front (rf) Feet Velocity
                                 ub_Prhxdot,   ub_Prhydot,   ub_Prhzdot,...Right Hind (rh) Feet Velocity
                                 ub_Flfx,      ub_Flfy,      ub_Flfz,...Left Front (lf) Feet Forces
                                 ub_Flhx,      ub_Flhy,      ub_Flhz,...Left Hind (lh) Feet Forces
                                 ub_Frfx,      ub_Frfy,      ub_Frfz,...Right Front (rf) Feet Forces
                                 ub_Frhx,      ub_Frhy,      ub_Frhz,...Right Hind (rh) Feet Forces
                                 ub_Ts];

%   Add lb/ub for Integer Variables if we perform Gait Discovery using
%   MINLP
if gait_discovery_switch == 1 % Discover Gait via MINLP
    lb_DecisionVars = [lb_ContinuousDecisionVars,...
                       lb_Clf,...
                       lb_Clh,...
                       lb_Crf,...
                       lb_Crh];
                   
    ub_DecisionVars = [ub_ContinuousDecisionVars,...
                       ub_Clf,...
                       ub_Clh,...
                       ub_Crf,...
                       ub_Crh];
elseif gait_discovery_switch == 2 % Motion Optimization using Fixed Gait
    lb_DecisionVars = lb_ContinuousDecisionVars;
    ub_DecisionVars = ub_ContinuousDecisionVars;
end