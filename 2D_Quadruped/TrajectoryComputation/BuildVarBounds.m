% Script for Building Variable Bounds

%   Build lower and upper bounds --> Follow the order Defined in
%   VarCategoryList
%   VarCategoryList = ["x",         "z",        "theta",...
%                      "xdot",      "zdot",     "thetadot",...
%                      "Plfx",      "Plfz",...
%                      "Plhx",      "Plhz",...
%                      "Prfx",      "Prfz",...
%                      "Prhx",      "Prhz",...
%                      "Plfxdot",   "Plfzdot",...
%                      "Plhxdot",   "Plhzdot",...
%                      "Prfxdot",   "Prfzdot",...
%                      "Prhxdot",   "Prhzdot",...
%                      "Flfx",      "Flfz",...
%                      "Flhx",      "Flhz",...
%                      "Frfx",      "Frfz",...
%                      "Frhx",      "Frhz",...
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
%       theta
lb_theta = repmat(-pi/2, 1, VarLengthList(VarCategoryList == "theta"));    
ub_theta = repmat( pi/2, 1, VarLengthList(VarCategoryList == "theta"));
%       xdot
lb_xdot = repmat(-5*speed, 1, VarLengthList(VarCategoryList == "xdot"));    
ub_xdot = repmat( 5*speed, 1, VarLengthList(VarCategoryList == "xdot"));
%       zdot
lb_zdot = repmat(-25, 1, VarLengthList(VarCategoryList == "zdot"));    
ub_zdot = repmat( 25, 1, VarLengthList(VarCategoryList == "zdot"));
%       thetadot
lb_thetadot = repmat(-5*pi, 1, VarLengthList(VarCategoryList == "thetadot"));    
ub_thetadot = repmat( 5*pi, 1, VarLengthList(VarCategoryList == "thetadot"));
%       Plfx
lb_Plfx = lb_x + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plfx"));  
ub_Plfx = ub_x + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plfx"));
%       Plfz
lb_Plfz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plfz"));  
ub_Plfz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plfz"));
%       Plhx
lb_Plhx = lb_x + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plhx"));  
ub_Plhx = ub_x + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Plhx"));
%       Plhz
lb_Plhz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plhz"));  
ub_Plhz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Plhz"));
%       Prfx
lb_Prfx = lb_x + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prfx"));  
ub_Prfx = ub_x + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prfx"));
%       Prfz
lb_Prfz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prfz"));  
ub_Prfz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prfz"));
%       Prhx
lb_Prhx = lb_x + repmat(-5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prhx"));  
ub_Prhx = ub_x + repmat( 5*BoundingBox_Width, 1, VarLengthList(VarCategoryList == "Prhx"));
%       Prhz
lb_Prhz = lb_z + repmat(-5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prhz"));  
ub_Prhz = ub_z + repmat( 5*BoundingBox_Height, 1, VarLengthList(VarCategoryList == "Prhz"));
%       Plfxdot
lb_Plfxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plfxdot"));
ub_Plfxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plfxdot"));
%       Plfzdot
lb_Plfzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plfzdot"));
ub_Plfzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plfzdot"));
%       Plhxdot
lb_Plhxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plhxdot"));
ub_Plhxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Plhxdot"));
%       Plhzdot
lb_Plhzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plhzdot"));
ub_Plhzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Plhzdot"));
%       Prfxdot
lb_Prfxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prfxdot"));
ub_Prfxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prfxdot"));
%       Prfzdot
lb_Prfzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prfzdot"));
ub_Prfzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prfzdot"));
%       Prhxdot
lb_Prhxdot = lb_xdot(1:end-1) + repmat(-5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prhxdot"));
ub_Prhxdot = ub_xdot(1:end-1) + repmat( 5*Vmax(1), 1, VarLengthList(VarCategoryList == "Prhxdot"));
%       Prhzdot
lb_Prhzdot = lb_zdot(1:end-1) + repmat(-5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prhzdot"));
ub_Prhzdot = ub_zdot(1:end-1) + repmat( 5*Vmax(2), 1, VarLengthList(VarCategoryList == "Prhzdot"));
%       Flfx
lb_Flfx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flfx"));    
ub_Flfx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flfx"));
%       Flfz
lb_Flfz = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flfz"));    
ub_Flfz = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flfz"));
%       Flhx
lb_Flhx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flhx"));    
ub_Flhx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Flhx"));
%       Flhz
lb_Flhz = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flhz"));    
ub_Flhz = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Flhz"));
%       Frfx
lb_Frfx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frfx"));    
ub_Frfx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frfx"));
%       Frfz
lb_Frfz = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frfz"));    
ub_Frfz = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frfz"));
%       Frhx
lb_Frhx = repmat(-1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frhx"));    
ub_Frhx = repmat( 1.5*Mf(1), 1, VarLengthList(VarCategoryList == "Frhx"));
%       Frhz
lb_Frhz = repmat(-1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frhz"));    
ub_Frhz = repmat( 1.5*Mf(2), 1, VarLengthList(VarCategoryList == "Frhz"));
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
lb_ContinuousDecisionVars =     [lb_x,         lb_z,        lb_theta,...
                                 lb_xdot,      lb_zdot,     lb_thetadot,...
                                 lb_Plfx,      lb_Plfz,...
                                 lb_Plhx,      lb_Plhz,...
                                 lb_Prfx,      lb_Prfz,...
                                 lb_Prhx,      lb_Prhz,...
                                 lb_Plfxdot,   lb_Plfzdot,...
                                 lb_Plhxdot,   lb_Plhzdot,...
                                 lb_Prfxdot,   lb_Prfzdot,...
                                 lb_Prhxdot,   lb_Prhzdot,...
                                 lb_Flfx,      lb_Flfz,...
                                 lb_Flhx,      lb_Flhz,...
                                 lb_Frfx,      lb_Frfz,...
                                 lb_Frhx,      lb_Frhz,...
                                 lb_Ts];
               
ub_ContinuousDecisionVars = [ub_x,         ub_z,        ub_theta,...
                             ub_xdot,      ub_zdot,     ub_thetadot,...
                             ub_Plfx,      ub_Plfz,...
                             ub_Plhx,      ub_Plhz,...
                             ub_Prfx,      ub_Prfz,...
                             ub_Prhx,      ub_Prhz,...
                             ub_Plfxdot,   ub_Plfzdot,...
                             ub_Plhxdot,   ub_Plhzdot,...
                             ub_Prfxdot,   ub_Prfzdot,...
                             ub_Prhxdot,   ub_Prhzdot,...
                             ub_Flfx,      ub_Flfz,...
                             ub_Flhx,      ub_Flhz,...
                             ub_Frfx,      ub_Frfz,...
                             ub_Frhx,      ub_Frhz,...
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