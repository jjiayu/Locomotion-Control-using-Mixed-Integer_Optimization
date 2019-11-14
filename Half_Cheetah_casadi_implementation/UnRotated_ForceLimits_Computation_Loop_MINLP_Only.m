%MINLP Computation Loop

import casadi.*

%=======================================================================
%
%   Big for-loop for to compute multiple MINLP Runs
%
for speedIdx = 1:length(SpeedList)
%for runIdx = 1:NumofRuns

%======================================================================
%   Parameter Setup Done, Actual Optimization Starts
%======================================================================
for runIdx = 1:NumofRuns
%for speedIdx = 1:length(SpeedList)
    
    %Scan the speed
    speed = SpeedList(speedIdx);
    
    diary off
    ExpLog_filename = strcat(['Speed-', num2str(speed), '-'], datestr(datetime('now'), 30)); %date format: 'yyyymmddTHHMMSS'(ISO 8601), e.g.20000301T154517
    diary([ExpDirectory, '/', ExpLog_filename]);
    
    disp('====================================================');
    disp(['Gait Search for ', num2str(NumPhases),'Phases Motion(', num2str(NumLocalKnots),'Knots per Phase),',' Locomotion Speed ', num2str(speed), ' m/s', ' Round ', num2str(runIdx)])
    disp(['Experiment Directory: ', ExpDirectory])
    disp('====================================================');

    %=======================================================================
    % Define/Create Modeling Variables
    %=======================================================================
    %   Display some Info
    disp('====================================================');
    disp('Generate Decision Variables Vectors/Lists:')
    disp('----------------------------------------------------');
    %-----------------------------------------------------------------------
    %   Continuous Variables
    %-----------------------------------------------------------------------
    %   State Variables: r = [x, y, theta, xdot, ydot, thetadot]
    %       x: horizontal position
    %       y: vertical position
    %       theta: torso orientation
    %       xdot: horizontal velocity
    %       ydot: vertical velocity
    %       thetadot: torso angular velocity
    %   Foot Step Locations (IN WORLD FRAME):
    %       Front Leg Location and Velocities: PF = [PFx, PFy, PFxdot, PFydot]
    %       Hind Leg Location and Velocities : PH = [PHx, PHy, PHxdot, PHydot]
    %   Foot-ground Reaction Forces (IN WORLD FRAME): 
    %       Front Leg Forces: FF = [FFx, FFy]
    %       Hind Leg Forces : FH = [FHx, FHy]
    %-----------------------------------------------------------------------
    %   Create CasADi SX variable Lists
    x = SX.sym('x', tauSeriesLength); 
    y = SX.sym('y', tauSeriesLength); 
    theta = SX.sym('theta', tauSeriesLength);
    xdot = SX.sym('xdot', tauSeriesLength); 
    ydot = SX.sym('ydot', tauSeriesLength); 
    thetadot = SX.sym('thetadot', tauSeriesLength);
    %           FootStep Locations and Velocities
    %               Front Leg Locations
    PFx = SX.sym('PFx', tauSeriesLength);
    PFy = SX.sym('PFy', tauSeriesLength);
    %               Front Leg Velocities
    PFxdot = SX.sym('PFxdot', tauSeriesLength - 1); 
    PFydot = SX.sym('PFydot', tauSeriesLength - 1);
    %               Hind Leg Locations
    PHx = SX.sym('PHx', tauSeriesLength);
    PHy = SX.sym('PHy', tauSeriesLength);
    %               Hind Leg Velocities
    PHxdot = SX.sym('PHxdot', tauSeriesLength - 1);
    PHydot = SX.sym('PHydot', tauSeriesLength - 1);
    %           Foot-Ground Reaction Forces
    %               Front Leg Forces
    FFx = SX.sym('FFx', tauSeriesLength - 1);
    FFy = SX.sym('FFy', tauSeriesLength - 1);
    %               Hind Leg Forces
    FHx = SX.sym('FHx', tauSeriesLength - 1);
    FHy = SX.sym('FHy', tauSeriesLength - 1);
    %-----------------------------------------------------------------------
    %       Create Decision Variable Name Lists 
    %           Using CreateVarsNameList(varsCasADi)
    %           varsCasADi is the CasADi variables
    %-----------------------------------------------------------------------
    x_label      = CreateVarsNameList(x);       y_label      = CreateVarsNameList(y);       theta_label    = CreateVarsNameList(theta);
    xdot_label   = CreateVarsNameList(xdot);    ydot_label   = CreateVarsNameList(ydot);    thetadot_label = CreateVarsNameList(thetadot);
    PFx_label    = CreateVarsNameList(PFx);     PFy_label    = CreateVarsNameList(PFy);    
    PFxdot_label = CreateVarsNameList(PFxdot);  PFydot_label = CreateVarsNameList(PFydot);
    PHx_label    = CreateVarsNameList(PHx);     PHy_label    = CreateVarsNameList(PHy);    
    PHxdot_label = CreateVarsNameList(PHxdot);  PHydot_label = CreateVarsNameList(PHydot);
    FFx_label    = CreateVarsNameList(FFx);     FFy_label    = CreateVarsNameList(FFy);
    FHx_label    = CreateVarsNameList(FHx);     FHy_label    = CreateVarsNameList(FHy);
    %-----------------------------------------------------------------------
    %   Phase-related Variables
    %-----------------------------------------------------------------------
    %       Note: For fixed switching time formulation and multi-phase formulation, 
    %             first create contact configuration and (siwtching time) from 0 to number of phases, and
    %             extract the sub variable list from 1 to number of phases
    %             The contact configuration and switching time variables should have the same length
    %             with respect to the number of phases
    %-----------------------------------------------------------------------
    %       Define CasADi Variables
    %-----------------------------------------------------------------------
    %       (*) Switching Time of Each Phase (Continuous Variable, Termination time of each phase)
    %-----------------------------------------------------------------------
    Ts = SX.sym('Ts', NumPhases + 1); %Ts is dented as Swtiching Time
    Ts = Ts(2:end);
    Ts_label = CreateVarsNameList(Ts);
    %-----------------------------------------------------------------------
    %       (*) Contact Configuration (Discrete Vairbale)
    %-----------------------------------------------------------------------
    %           Contact Configuration (Mode)
    %               Leg Contact Configurations: C = [CF, CH]; CF, CH = 0/1 (Binary Variable)
    %                   Front Leg Contact On/Off: CF
    %                   Hind Leg Contact On/Off : CH
    %--------------------------------------------
    CF = SX.sym('CF', NumPhases + 1);
    CH = SX.sym('CH', NumPhases + 1);
    CF = CF(2:end);
    CH = CH(2:end);

    %       Create Variable Name List
    CF_label = CreateVarsNameList(CF);
    CH_label = CreateVarsNameList(CH);
    %----------------------------------------------------------------------
    %       Check Correctness of the Generated Variables
    %----------------------------------------------------------------------
    if length(CF) == NumPhases && length(CH) == NumPhases && length(Ts) == NumPhases
        disp('Checked - Contact Configuration and Switching Time Varaibels for Each Phase are Generated Coorectly')
    else
        ME_PhaseRelatedConfig = MException('Initialization:PhaseRelatedConfig','The contact configuration and switching time is not defined for each phase');
        throw(ME_PhaseRelatedConfig)
    end

    %-----------------------------------------------------------------------
    %   Assemble Lists for all decision variables
    %       Identifiers of all variables
    varList = ["x",    "y",      "theta",...
               "xdot", "ydot",   "thetadot",...
               "PFx",  "PFy",    "PFxdot",      "PFydot",...
               "PHx",  "PHy",    "PHxdot",      "PHydot",...
               "FFx",  "FFy",...
               "FHx",  "FHy",...
               "Ts",...
               "CF",   "CH"];

    %       Variable Length List
    VarLengthList = [length(x_label),     length(y_label),      length(theta_label), ...
                     length(xdot_label),  length(ydot_label),   length(thetadot_label), ...
                     length(PFx_label),   length(PFy_label),    length(PFxdot_label),       length(PFydot_label), ...
                     length(PHx_label),   length(PHy_label),    length(PHxdot_label),       length(PHydot_label), ...
                     length(FFx_label),   length(FFy_label),...
                     length(FHx_label),   length(FHy_label),...
                     length(Ts_label),...
                     length(CF_label),    length(CH_label)];

    %       Full decision variable names list
    VarNamesList = [x_label,      y_label,      theta_label,...
                    xdot_label,   ydot_label,   thetadot_label,...
                    PFx_label,    PFy_label,    PFxdot_label,      PFydot_label,...
                    PHx_label,    PHy_label,    PHxdot_label,      PHydot_label,...
                    FFx_label,    FFy_label,...
                    FHx_label,    FHy_label,...
                    Ts_label,...
                    CF_label,     CH_label];
    %----------------------------------------------------------------------
    %   Verifications
    %       (Place Holder - Not Required) Check if variable names are defined
    disp('-----------------------------------------------------------')
    %=======================================================================

    %=======================================================================
    % Symbolic Functions
    %=======================================================================
    %   Euler Integration: x_k+1 - x_k = h*fdyn_k -->
    %                      x_k+1 - x_k - h*fdyn_k = 0
    %-----------------------------------------------------------------------
    %       Variable Definitions:
    xk  = SX.sym('x[k]');                  %xk, current state
    xkk = SX.sym(['x[k+',num2str(1),']']); %xk+1 next state
    fdyn_k  = SX.sym('fdyn[k]');               %fdyn_k, dynamical equation value evaluated at time step k
    hk  = SX.sym('h[k]');                  %hk, current time step length
    %       Build Function
    euler = xkk - xk - hk*fdyn_k; %expression
    EulerIntegration = Function('Euler',{xkk,xk,hk,fdyn_k},{euler}); %function, can take either symbolic or numerical inputs
    %-----------------------------------------------------------------------
    %   Complementarity Constraint (a Set of Inequality Equations):
    %-----------------------------------------------------------------------
    %       Variable Definitions:
    vk   = SX.sym('v[k]'); %vk, continuous decision variable
    zk   = SX.sym('z[k]'); %zk, integer variable
    bigM = SX.sym('bigM'); % big-M
    %       Build Functions
    %           Type 1
    %           Difference --> v[k] - bigM*z[k] >=/<= Constant
    ineq_diff = vk - bigM*zk;
    Ineq_Difference = Function('Ineq_Difference', {vk, bigM, zk}, {ineq_diff});
    %           Type 2
    %           Summation --> v[k] + bigM*z[k] >=/<= Constant
    ineq_sum  = vk + bigM*zk;
    Ineq_Summation  = Function('Ineq_Summation',  {vk, bigM, zk}, {ineq_sum});
    %-----------------------------------------------------------------------
    %   Kinematics Constraint -->
    %          -1/2*b (Constant Vector) <= R(theta[k])(P[k]-r[k])- Pcenter <= 1/2*b (Constant Vector)
    %                       b = [bw;bh];                                                                                                                                                                                                         
    %--------------------------
    %       (Place Holder) Need to Augment when moving to 3D
    %--------------------------
    %       Variable Definitions
    xk     = SX.sym('x[k]');
    yk     = SX.sym('y[k]');
    thetak = SX.sym('theta[k]');
    rk     = [xk,yk,thetak];
    PcX    = SX.sym('PcenterX'); %Foot/End-Effector default x-axis position
    PcY    = SX.sym('PcenterY'); %Foot/End-Effector default y-axis position
    Pc     = [PcX,PcY]; %Foot/End-Effector default positions
    Pxk    = SX.sym('Px[k]');
    Pyk    = SX.sym('Py[k]');
    Pk     = [Pxk,Pyk];
    %       Build Function
    kinematics = [cos(thetak), -sin(thetak); sin(thetak), cos(thetak)]'*(Pk' - [xk,yk]') - Pc';
    KinematicsConstraint = Function('KinematicsConstraint',{rk, Pk, Pc},{kinematics});
    %-----------------------------------------------------------------------
%     %   Friction Cones (In 2D, Flat Terrain) -->
%     %               Fx[k] - Const_miu*(Norm*[Fx[k],Fy[k]]') <= 0
%     %--------------------------------
%     %   (Place Holder) Need to Change when move to 3D Case and/or Uneven
%     %   Terrains (change equation and norm)
%     %--------------------------------
%     %       Variable Definitions
%     Const_miu = SX.sym('miu');
%     NormX     = SX.sym('Nx[k]');
%     NormY     = SX.sym('Ny[k]');
%     Norm      = [NormX, NormY];
%     ForceX    = SX.sym('Fx[k]');
%     ForceY    = SX.sym('Fy[k]');
%     Force     = [ForceX, ForceY];
%     %       Build Functions
%     friction = ForceX - Const_miu*(dot(Norm,Force));
%     FrictionCone = Function('FrictionCone',{Norm, Force, Const_miu},{friction});
    %-----------------------------------------------------------------------

    %=======================================================================
    % Build Constraints and Objective Function
    %=======================================================================   
    %   Initialize Constraints and Cost Function Constainers
    %-----------------------------------------------------------------------
    %       Collect all decision varibales
    DecisionVars = {x,          y,          theta,...
                    xdot,       ydot,       thetadot,...
                    PFx,        PFy,        PFxdot,         PFydot,...
                    PHx,        PHy,        PHxdot,         PHydot,...
                    FFx,        FFy,...
                    FHx,        FHy,...
                    Ts,...
                    CF,         CH}; 
    DecisionVars = vertcat(DecisionVars{:}); %make a vertical vector

    %       Verify if the variable names are consistent in DecisionVars and VarNameList 
    if length(DecisionVars) == length(VarNamesList)
        for i = 1:length(DecisionVars)
            temp = DecisionVars(i);
            if temp.name() ~= VarNamesList(i)
                ME_NameListInconsistent = MException('Initialization:InconsistentNameList',['The variable names in DecisionVars and VarNamesList are different at index ', num2str(i)]);
                throw(ME_NameListInconsistent)
            end
        end
        disp('Checked - Names in DecisionVars are consistent with the name labels in VarNamesList');
    else
        ME_NameListLengthInconsistent= MException('Initialization:NameListInconsistent','DecisionVars has different size with respect to VarNamesList');
        throw(ME_NameListLengthInconsistent)
    end

    %       Other Important Setups
%     %           Initial Guess of Decision Variables
%     DecisionVarsInit = 10*rand(size(DecisionVars)); %Random Initial Guess
    %DecisionVarsInit = zeros(size(DecisionVars)); %Zero Initial guess
    %           Lower and upper bounds, variable type
    lb_DecisionVars = [];  %Variable Lower Bound 
    ub_DecisionVars = [];  %Variable Upper Bound
    varstype        = [];  %Variable Type: 0 -> Continuous Variable/ 1 -> Binary Variable
    for i = 1:length(varList)
        if contains(varList(i),'C') == 1 %binary variable
            lb_DecisionVars = [lb_DecisionVars, zeros(1, VarLengthList(i))];    
            ub_DecisionVars = [ub_DecisionVars, ones( 1, VarLengthList(i))];
            varstype        = [varstype,        ones(1, VarLengthList(i))];            %0 (use zeros) -> Continuous Variable/ 1 (use ones) -> Binary Variable
        else %continuous variables
            if     strcmp(varList(i),'x') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-0.1, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( speed*Tend+0.1*speed*Tend, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'y') == 1
                if TerrainType == 1 %Flat Terrain
                    lb_DecisionVars = [lb_DecisionVars, repmat(-5, 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( 5, 1, VarLengthList(i))];
                elseif TerrainType == 2 %slope
                    if MaxSpeed*Tend >= 5
                        baseheightlimit = 2*MaxSpeed*Tend;
                    else
                        baseheightlimit = 5;
                    end
                    lb_DecisionVars = [lb_DecisionVars, repmat(-baseheightlimit/cos(terrain_slope_rad), 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( baseheightlimit/cos(terrain_slope_rad), 1, VarLengthList(i))];
                end
            elseif strcmp(varList(i),'theta') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-pi/2, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( pi/2, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'xdot') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-5*speed, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( 5*speed, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'ydot') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-25, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( 25, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'thetadot') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-5*pi, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( 5*pi, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'PFx') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-speed*Tend - 5*BoundingBox_Width, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( speed*Tend + 5*BoundingBox_Width, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'PFy') == 1
                if TerrainType == 1 %flat terrain
                    lb_DecisionVars = [lb_DecisionVars, repmat(-5 - 5*BoundingBox_Height, 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( 5                       , 1, VarLengthList(i))];
                elseif TerrainType == 2 %slope
                    if MaxSpeed*Tend >= 5
                        baseheightlimit = 2*MaxSpeed*Tend;
                    else
                        baseheightlimit = 5;
                    end
                    lb_DecisionVars = [lb_DecisionVars, repmat(-baseheightlimit/cos(terrain_slope_rad) - 5*BoundingBox_Height, 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( baseheightlimit/cos(terrain_slope_rad)                       , 1, VarLengthList(i))];
                end
            elseif strcmp(varList(i),'PHx') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-speed*Tend - 5*BoundingBox_Width, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( speed*Tend + 5*BoundingBox_Width, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'PHy') == 1
                if TerrainType == 1 %Flat Terrain
                    lb_DecisionVars = [lb_DecisionVars, repmat(-5 - 5*BoundingBox_Height, 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( 5                       , 1, VarLengthList(i))];
                elseif TerrainType == 2 %slope
                    if MaxSpeed*Tend >= 5
                        baseheightlimit = 2*MaxSpeed*Tend;
                    else
                        baseheightlimit = 5;
                    end
                    lb_DecisionVars = [lb_DecisionVars, repmat(-baseheightlimit/cos(terrain_slope_rad) - 5*BoundingBox_Height, 1, VarLengthList(i))];    
                    ub_DecisionVars = [ub_DecisionVars, repmat( baseheightlimit/cos(terrain_slope_rad)                       , 1, VarLengthList(i))];
                end
            %elseif strcmp(varList(i),'PFxdot') == 1
                %lb_DecisionVars = [lb_DecisionVars, repmat(-Mvelx - 0.1*Mvelx, 1, VarLengthList(i))];    
                %ub_DecisionVars = [ub_DecisionVars, repmat( Mvelx + 0.1*Mvelx, 1, VarLengthList(i))];
                %lb_DecisionVars = [lb_DecisionVars, repmat(-50, 1, VarLengthList(i))];    
                %ub_DecisionVars = [ub_DecisionVars, repmat( 50, 1, VarLengthList(i))];
            %elseif strcmp(varList(i),'PFydot') == 1
            %    lb_DecisionVars = [lb_DecisionVars, repmat(-Mvely - 0.1*Mvely, 1, VarLengthList(i))];    
            %    ub_DecisionVars = [ub_DecisionVars, repmat( Mvely + 0.1*Mvely, 1, VarLengthList(i))];
            %     lb_DecisionVars = [lb_DecisionVars, repmat(-50, 1, VarLengthList(i))];    
            %     ub_DecisionVars = [ub_DecisionVars, repmat( 50, 1, VarLengthList(i))];
            %elseif strcmp(varList(i),'PHxdot') == 1
            %    lb_DecisionVars = [lb_DecisionVars, repmat(-Mvelx - 0.1*Mvelx, 1, VarLengthList(i))];    
            %    ub_DecisionVars = [ub_DecisionVars, repmat( Mvelx + 0.1*Mvelx, 1, VarLengthList(i))];
            %     lb_DecisionVars = [lb_DecisionVars, repmat(-50, 1, VarLengthList(i))];    
            %     ub_DecisionVars = [ub_DecisionVars, repmat( 50, 1, VarLengthList(i))];
            %elseif strcmp(varList(i),'PHydot') == 1
            %    lb_DecisionVars = [lb_DecisionVars, repmat(-Mvely - 0.1*Mvely, 1, VarLengthList(i))];    
            %    ub_DecisionVars = [ub_DecisionVars, repmat( Mvely + 0.1*Mvely, 1, VarLengthList(i))];
            %     lb_DecisionVars = [lb_DecisionVars, repmat(-50, 1, VarLengthList(i))];    
            %     ub_DecisionVars = [ub_DecisionVars, repmat( 50, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'FFx') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-Mfx - 0.1*Mfx, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( Mfx + 0.1*Mfx, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'FHx') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-Mfx - 0.1*Mfx, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( Mfx + 0.1*Mfx, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'FFy') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-Mfy - 0.1*Mfy, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( Mfy + 0.1*Mfy, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'FHy') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat(-Mfy - 0.1*Mfy, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( Mfy + 0.1*Mfy, 1, VarLengthList(i))];
            elseif strcmp(varList(i),'Ts') == 1
                lb_DecisionVars = [lb_DecisionVars, repmat( 0, 1, VarLengthList(i))];  
                %lb_DecisionVars = [lb_DecisionVars, repmat( 0.01*Tend, 1, VarLengthList(i))];  
                ub_DecisionVars = [ub_DecisionVars, repmat( Tend+0.1*Tend, 1, VarLengthList(i))];
            else %other unbounded variables
                lb_DecisionVars = [lb_DecisionVars, repmat(-inf, 1, VarLengthList(i))];    
                ub_DecisionVars = [ub_DecisionVars, repmat( inf, 1, VarLengthList(i))];
            end

            varstype        = [varstype,        zeros(1, VarLengthList(i))];           %0 (use zeros) -> Continuous Variable/ 1 (use ones) -> Binary Variable
        end
    end
    
    %   Generate Initial Guess that samples the entire search space
    DecisionVarsInit = zeros(size(DecisionVars));
    %DecisionVarsInit(find(VarNamesList == x_label(1)):find(VarNamesList == x_label(end))) = linspace(0,speed*Tend,length(x_label));
    %DecisionVarsInit(find(VarNamesList == y_label(1)):find(VarNamesList == Ts_label(end))) = lb_DecisionVars(find(VarNamesList == y_label(1)):find(VarNamesList == Ts_label(end))) + (ub_DecisionVars(find(VarNamesList == y_label(1)):find(VarNamesList == Ts_label(end)))-lb_DecisionVars(find(VarNamesList == y_label(1)):find(VarNamesList == Ts_label(end)))).*rand(1,length(find(VarNamesList == y_label(1)):find(VarNamesList == Ts_label(end))));
    DecisionVarsInit(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end))) = lb_DecisionVars(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end))) + (ub_DecisionVars(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end)))-lb_DecisionVars(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end)))).*rand(1,length(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end))));
    DecisionVarsInit(find(VarNamesList == CF_label(1)):find(VarNamesList == CH_label(end))) = randi([0,1],length(find(VarNamesList == CF_label(1)):find(VarNamesList == CH_label(end))),1);
    DecisionVarsInit(find(VarNamesList == PFxdot_label(1)):find(VarNamesList == PFydot_label(end))) = -5 + (10)*rand(1,length(find(VarNamesList == PFxdot_label(1)):find(VarNamesList == PFydot_label(end))));
    DecisionVarsInit(find(VarNamesList == PHxdot_label(1)):find(VarNamesList == PHydot_label(end))) = -5 + (10)*rand(1,length(find(VarNamesList == PHxdot_label(1)):find(VarNamesList == PHydot_label(end))));    %DecisionVarsInit(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end))) = lb_DecisionVars(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end))) + rand(1, length(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end)))).*ub_DecisionVars(find(VarNamesList == x_label(1)):find(VarNamesList == Ts_label(end)));
    %DecisionVarsInit(find(VarNamesList == CF_label(1)):find(VarNamesList == CH_label(end))) = randi([0,1],length(find(VarNamesList == CF_label(1)):find(VarNamesList == CH_label(end))),1);
    %DecisionVarsInit(find(VarNamesList == PFxdot_label(1)):find(VarNamesList == PHydot_label(end))) = 5*rand(1,length(find(VarNamesList == PFxdot_label(1)):find(VarNamesList == PHydot_label(end))));
    
    %------------------------
    %           (Place Holder) Check the size of lower and upper bounds and
    %           vartype with respect the DecisionVars
    %------------------------
    %           container of constraints
    g   = {}; 
    %           Upper Bound of Constraint Functions
    lbg = []; 
    %           Lower Bound of Constraint Functions
    ubg = []; 
    %           Cost Function
    J   = 0;  
    %-----------------------------------------------------------------------
    % Generate Constraints and Cost Function
    %-----------------------------------------------------------------------
    %   Create Time Step Variable for each Phase
    hVector = [Ts(1)-0];
    hVector = tauStepLength*NumPhases*[hVector;diff(Ts)];
    %-----------------------------------------------------------------------
    %   Loop Over the Time Horizon
    %-----------------------------------------------------------------------
    for k = 1:tauSeriesLength

        %--------------------------------------
        % Extract Phase Index
        if k<= tauSeriesLength - 1
            PhaseIdx = floor((k-1)/NumLocalKnots) + 1; % k-1 is the time step enumeration
        elseif k == tauSeriesLength
            PhaseIdx = floor((k-1)/NumLocalKnots) + 1 - 1; %the equation will give last time belongs to NumPhases +1, we simply classify it into the last phase
        end
        %--------------------------------------

        %----------------------------------
        % Get time step
        h = hVector(PhaseIdx);

        %--------------------------------------
        % System dynamics
        %   Use EulerIntegration Function to Construct
        %   EulerIntegration = x[k+1] - x[k] -h[k]*fdyn[k] = 0
        %   Input: x[k+1], x[k], h[k], fdyn[k] (Display sequence as defined in EulerIntegration Function)
        %   lbg = 0 (Equality constraint)
        %   ubg = 0 (Equality constraint)
        %   Covering Range: from k = 1 to k = tauSeriesLength - 1
        %--------------------------------------
        %   Constraint the range
        %--------------------------------------
        if k <= tauSeriesLength - 1
            %----------------------------------
            % Robot Torso Dynamics
            %--------------------------------------
            % (*) x-axis first-order dynamics (position)
            %       Equation: x[k+1] - x[k] - h*xdot[k] = 0
            %       Input: x[k+1]  = x[k+1] (x[k+1] at rhs means x-position)
            %              x[k]    = x[k]   (x[k] at rhs means x-position)
            %              h[k]    = h
            %              fdyn[k] = xodt[k]
            %       lbg = 0
            %       ubg = 0
            EqTemp = EulerIntegration(x(k+1), x(k), h, xdot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %----------------------------------------
            % (*) x-axis second-order dynamics (velocity)
            %       Equation: xdot[k+1] - xdot[k] - h*(1/m*FFx[k] + 1/m*FHx[k])
            %       Input: x[k+1]  = xdot[k+1]
            %              x[k]    = xdot[k]
            %              h[k]    = h
            %              fdyn[k] = 1/m*(FFx[k]+FHx[k])
            %       lbg = 0;
            %       ubg = 0;
            fdyntemp  = 1/m*(FFx(k)+FHx(k));
            EqTemp    = EulerIntegration(xdot(k+1), xdot(k), h, fdyntemp);
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) y-axis first-order dynamics (position)
            %       Equation: y[k+1] - y[k] - h[k]*ydot[k] = 0
            %       Input: x[k+1]  = y[k+1]
            %              x[k]    = y[k]
            %              h[k]    = h
            %              fdyn[k] = ydot[k]
            EqTemp = EulerIntegration(y(k+1), y(k), h, ydot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) y-axis second-order dynamics (velocity)
            %       Equation: ydot[k+1] - ydot[k] - h[k]*[1/m*(FFy[k]+FHy[k]-g)] = 0
            %       Input: x[k+1]  = ydot[k+1]
            %              x[k]    = ydot[k]
            %              h[k]    = h
            %              fdyn[k] = 1/m*(FFy[k] + FHy[k] - g)
            %       lbg = 0
            %       ubg = 0
            fdyntemp = 1/m*(FFy(k) + FHy(k)) - G;
            EqTemp   = EulerIntegration(ydot(k+1), ydot(k), h, fdyntemp);
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) theta first-order dynamics (position)
            %       Equation: theta[k+1] - theta[k] - h[k]*thetadot[k] = 0
            %       Input: x[k+1]  = theta[k+1]
            %              x[k]    = theta[k]
            %              h[k]    = h
            %              fdyn[k] = thetadot[k]
            %       lbg = 0
            %       ubg = 0
            EqTemp = EulerIntegration(theta(k+1), theta(k), h, thetadot(k));
            g   = {g{:}, EqTemp};   %Append to constraint function list
            lbg = [lbg;  0];        %Give constraint lower bound
            ubg = [ubg;  0];        %Give constraint upper bound
            %-----------------------------------------
            % (*) theta second-order dynamics (velocity)
            %       Equation: Izz*thetadot[k+1] - Izz*thetadot[k] - h[k]*[(PFx[k] - x[k])*FFy[k] - (PFy[k] - y[k])*FFx[k] + (PHx[k] - x[k])*FHy[k] - (PHy[k] - y[k])*FHx[k]]
            %       Input: x[k+1]  = Izz*thetadot[k+1]
            %              x[k]    = Izz*thetadot[k]
            %              h[k]    = h
            %              fdyn[k] = (PFx[k] - x[k])*FFy[k] - (PFy[k] - y[k])*FFx[k] + (PHx[k] - x[k])*FHy[k] - (PHy[k] - y[k])*FHx[k]
            %       lbg = 0
            %       ubg = 0
            fdyntemp = (PFx(k) - x(k))*FFy(k) - (PFy(k) - y(k))*FFx(k) + (PHx(k) - x(k))*FHy(k) - (PHy(k) - y(k))*FHx(k);
            EqTemp   = EulerIntegration(I*thetadot(k+1), I*thetadot(k), h, fdyntemp);
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % Footstep Location dynamics
            %-----------------------------------------
            % (*) PFx (Front foot x-axis) first-order dynamics (velocity)
            %       Equation: PFx[k+1] - PFx[k] - h[k]*PFxdot[k] = 0
            %       Input: x[k+1]  = PFx[k+1]
            %              x[k]    = PFx[k]
            %              h[k]    = h
            %              fdyn[k] = PFxdot[k]
            %       lbg = 0
            %       ubg = 0
            EqTemp = EulerIntegration(PFx(k+1), PFx(k), h, PFxdot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) PFy (Front foot y-axis) first-order dynamics (velocity)
            %       Euqation: PFy[k+1] - PFy[k] - h[k]*PFydot[k] = 0
            %       Input: x[k+1]  = PFy[k+1]
            %              x[k]    = PFy[k]
            %              h[k]    = h
            %              fdyn[k] = PFydot[k]
            %       lbg = 0;
            %       ubg = 0;
            EqTemp = EulerIntegration(PFy(k+1), PFy(k), h, PFydot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) PHx (Hind foot x-axis) first-order dynamics (velocity)
            %       Equation: PHx[k+1] - PHx[k] - h[k]*PHxdot[k] = 0
            %       Input: x[k+1]  = PHx[k+1]
            %              x[k]    = PHx[k]
            %              h[k]    = h
            %              fdyn[k] = PHxdot[k]
            %       lbg = 0;
            %       ubg = 0;
            EqTemp = EulerIntegration(PHx(k+1), PHx(k), h, PHxdot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            % (*) PHy (Hind foot y-axis) first-order dynamics (velocity)
            %       Equation: PHy[k+1] - PHy[k] - h[k]*PHydot[k] = 0
            %       Input: x[k+1]  = PHy[k+1]
            %              x[k]    = PHy[k]
            %              h[k]    = h
            %              fdyn[k] = PHydot[k]
            %       lbg = 0
            %       ubg = 0
            EqTemp = EulerIntegration(PHy(k+1), PHy(k), h, PHydot(k));
            g   = {g{:}, EqTemp};  %Append to constraint function list
            lbg = [lbg;  0];       %Give constraint lower bound
            ubg = [ubg;  0];       %Give constraint upper bound
            %-----------------------------------------
            %   Dynamical Equation Constraint Setup - Done

            %----------------------------------------------------
            % Complementarity Constraint
            %   Use Functions: Ineq_Difference --> v[k] - bigM*z[k]
            %                  Ineq_Summation  --> v[k] + bigM*z[k]
            %   Input: v[k] --> Continuous Variables at time step k
            %          bigM --> bigM constant
            %          z[k] --> Integer/Binary Variable at time step k
            %----------------------------------------------------
        %         %   (*) Extract Index for Identifying Contact Configuration for
        %         %--------------------------------------------------------------
        %         %   Governing Current Time Step
        % 
        %             ContactConfigIdx = floor((k-1)/NumLocalTimeSteps) + 1; % k-1 is the time step enumeration

            %----------------------------------------------------
            %   (*) Foot/End-effector Position (y-axis only)
            %---------------------------------------------------
            %       (Place Holder) Need to Change Height into loop-up table
            %       function if we go for uneven terrain
            %--------------------------------------------------
            %       - Equation (1): Py <= Height + Mpos(1-C) -->
            %                       Py + Mpos*C <= Height + Mpos -->
            %                       Py + Mpos*C - Height(x) <= Mpos
            %         Use Ineq_Summation
            %         Input: v[k] = P(F/H)y[k]
            %                bigM = Mpos_y
            %                z[k] = C(F/H)[k]
            %                Additionally: - TerrainHeight
            %         lbg = -inf 
            %         ubg = Mpos_y
            %-------------------------------------------    
            %           Front Leg
                EqTemp = Ineq_Summation(PFy(k),Mpos_y, CF(PhaseIdx)) - TerrainModel(PFx(k));
                g   = {g{:}, EqTemp};                      %Append to constraint function list
                lbg = [lbg;  -inf];                        %Give constraint lower bound
                ubg = [ubg;  Mpos_y];      %Give constraint upper bound

            %           Hind Leg
                EqTemp = Ineq_Summation(PHy(k), Mpos_y, CH(PhaseIdx)) - TerrainModel(PHx(k));
                g   = {g{:}, EqTemp};                      %Append to constraint function list
                lbg = [lbg;  -inf];                        %Give constraint lower bound
                ubg = [ubg;  Mpos_y];      %Give constraint upper bound
            %-------------------------------------------
            %       - Equation (2): Py >= Height -->
            %                       Height <= Py <= inf -->
            %       For Uneven Terrain: 0 <= Py -Height(x) <= inf
            %
            %----------------------------------------------------
            %         (Place Holder) For even terrain, achieve this constraint by changing variable lower bounds
            %                        Change to complementarity form when
            %                        introducing uneven terrain
            %----------------------------------------------------
            %   Front Leg
            EqTemp = PFy(k) - TerrainModel(PFx(k));
            g   = {g{:}, EqTemp};
            lbg = [lbg;  0];
            ubg = [ubg;  inf];
            %   Hind Leg
            EqTemp = PHy(k) - TerrainModel(PHx(k));
            g   = {g{:}, EqTemp};
            lbg = [lbg;  0];
            ubg = [ubg;  inf];
            
            %-------------------------------------------------------
            %       Feet Velocity Constraint
            %-------------------------------------------------------
            %       Set One: Big-M Openinig and Closing of Feet Velocity
            %           Equations (1): Pdot <= Mvel(1-C)   -->
            %                          Pdot <= Mvel - Mvel*C -->
            %                          Pdot + Mvel*C <= Mvel
            %           lbg = -inf
            %           ubg = Mvel
                        %Mvel is a big
            %           enough constant value (100) to control the opening
            %           and closing of the constraint
            %--------------------------------------------------------
            %           Front Leg x-axis    
                EqTemp = PFxdot(k) + Mvel*CF(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -inf];
                ubg = [ubg; Mvel];
            %           Front Leg y-axis
                EqTemp = PFydot(k) + Mvel*CF(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -inf];
                ubg = [ubg; Mvel];
            %           Hind Leg x-axis    
                EqTemp = PHxdot(k) + Mvel*CH(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -inf];
                ubg = [ubg; Mvel];
            %           Hind Leg y-axis
                EqTemp = PHydot(k) + Mvel*CH(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -inf];
                ubg = [ubg; Mvel];
            %-----------------------------------------------------------
            %       Equation (2): Pdot >= -Mvel(1-C) -->
            %                     Pdot >= -Mvel + Mvel*C -->
            %                     Pdot - Mvel*C >= -Mvel
            %       lbg = -Mvel
            %       ubg = inf
            %-----------------------------------------------------------
            %           Front Leg x-axis
                EqTemp = PFxdot(k) - Mvel*CF(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvel];
                ubg = [ubg; inf];
            %           Front Leg y-axis
                EqTemp = PFydot(k) - Mvel*CF(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvel];
                ubg = [ubg; inf];
            %           Hind Leg x-axis
                EqTemp = PHxdot(k) - Mvel*CH(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvel];
                ubg = [ubg; inf];
            %           Hind Leg y-axis
                EqTemp = PHydot(k) - Mvel*CH(PhaseIdx);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvel];
                ubg = [ubg; inf];
            %---------------------------------------------------------
            %       Set (2): Speed Ranges
            %---------------------------------------------------------
            %           Equation :   |Pdot - Pdot_body| <= Mvel_x/y -->
            %                 -Mvel_x/y <= Pdot - Pdot_body <= Mvel_x/y
            %           lbg = -inf
            %           ubg = Mvel_x/y
            %---------------------------------------------------------
            %           Front Leg x-axis
                EqTemp = cos(theta(k))*(PFxdot(k) - xdot(k)) + sin(theta(k))*(PFydot(k) - ydot(k));
                %EqTemp = PFxdot(k) - xdot(k);
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvelx];
                ubg = [ubg; Mvelx];
            %           Front Leg y-axis
                EqTemp = -sin(theta(k))*(PFxdot(k) - xdot(k)) + cos(theta(k))*(PFydot(k) - ydot(k));
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvely];
                ubg = [ubg; Mvely];
            %           Hind Leg x-axis
                EqTemp = cos(theta(k))*(PHxdot(k) - xdot(k)) + sin(theta(k))*(PHydot(k) - ydot(k));
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvelx];
                ubg = [ubg; Mvelx];
            %           Hind Leg y-axis
                EqTemp = -sin(theta(k))*(PHxdot(k) - xdot(k)) + cos(theta(k))*(PHydot(k) - ydot(k));
                g   = {g{:}, EqTemp};
                lbg = [lbg; -Mvely];
                ubg = [ubg; Mvely];
%-------------------------------------------------------------------------
%   Old Version - In-Correct for sure but the rotation matrix may be useful
%-------------------------------------------------------------------------
%             %       - Equations (1): Body_Velo + R(theta)Pdot <= 0 + Mvel(1-C) -->
%             %                       R(theta)Pdot + Mvel*C <= Mvel -->
%             %         Use Ineq_Summation
%             %         Input: v[k] = P(F/H)(x/y)dot[k]
%             %                bigM = Mvel
%             %                z[k] = C(F/H)[k]
%             %         lbg = -inf
%             %         ubg = Mvel
%             %----------------------------------------------------
%             %           Front Leg x-axis
%                 EqTemp = Ineq_Summation(cos(theta(k))*PFxdot(k)+sin(theta(k))*PFydot(k), Mvelx, CF(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -inf];                        %Give constraint lower bound
%                 ubg = [ubg;  Mvelx];                       %Give constraint upper bound
% 
%             %           Front Leg y-axis
%                 EqTemp = Ineq_Summation(-sin(theta(k))*PFxdot(k)+cos(theta(k))*PFydot(k), Mvely, CF(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -inf];                        %Give constraint lower bound
%                 ubg = [ubg;  Mvely];                       %Give constraint upper bound
% 
%             %           Hind Leg x-axis
%                 EqTemp = Ineq_Summation(cos(theta(k))*PHxdot(k)+sin(theta(k))*PHydot(k), Mvelx, CH(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -inf];                        %Give constraint lower bound
%                 ubg = [ubg;  Mvelx];                       %Give constraint upper bound
% 
%             %           Hind Leg y-axis
%                 EqTemp = Ineq_Summation(-sin(theta(k))*PHxdot(k)+cos(theta(k))*PHydot(k), Mvely, CH(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -inf];                        %Give constraint lower bound
%                 ubg = [ubg;  Mvely];                       %Give constraint upper bound
%             %------------------------------------------------------              
%             %       - Equation (2): R(theta)Pdot >= 0 - Mvel(1-C) -->
%             %                       R(theta)Pdot - Mvel*C >= -Mvel -->
%             %                       -Mvel <= R(theta)Pdot - Mvel*C
%             %         Use Function Ineq_Difference
%             %         Input: v[k] = P(F/H)(x/y)dot[k]
%             %                bigM = -Mvel
%             %                z[k] = C(F/H)[k]
%             %         lbg = -Mvel
%             %         ubg = inf
%             %------------------------------------------------------
%             %           Front Leg x-axis
%                 EqTemp = Ineq_Difference(cos(theta(k))*PFxdot(k)+sin(theta(k))*PFydot(k), Mvelx, CF(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -Mvelx];                      %Give constraint lower bound
%                 ubg = [ubg;  inf];                         %Give constraint upper bound
% 
%             %           Front Leg y-axis
%                 EqTemp = Ineq_Difference(-sin(theta(k))*PFxdot(k)+cos(theta(k))*PFydot(k), Mvely, CF(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -Mvely];                      %Give constraint lower bound
%                 ubg = [ubg;  inf];                         %Give constraint upper bound
% 
%             %           Hind Leg x-axis
%                 EqTemp = Ineq_Difference(cos(theta(k))*PHxdot(k)+sin(theta(k))*PHydot(k), Mvelx, CH(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -Mvelx];                      %Give constraint lower bound
%                 ubg = [ubg;  inf];                         %Give constraint upper bound
% 
%             %           Hind Leg y-axis
%                 EqTemp = Ineq_Difference(-sin(theta(k))*PHxdot(k)+cos(theta(k))*PHydot(k), Mvely, CH(PhaseIdx));
%                 g   = {g{:}, EqTemp};                      %Append to constraint function list
%                 lbg = [lbg;  -Mvely];                      %Give constraint lower bound
%                 ubg = [ubg;  inf];                         %Give constraint upper bound
            %------------------------------------------------------
            %   (*) Foot/End-Effector Forces
            %------------------------------------------------------
            %     (-) x-axis
            %-----------------------------------------------------
            %       - Euqation (1): Fx <= 0 + Mfx*C -->
            %                     Fx - Mfx*C <= 0
            %           Use Ineq_Difference
            %           Input: v[k] = F(F/H)x[k]
            %                  bigM = Mfx
            %                  z[k] = C(F/H)[k]
            %           lbg = -inf
            %           ubg = 0
            %-----------------------------------------------------
            %           Front Leg
                EqTemp = Ineq_Difference(FFx(k), Mfx, CF(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];       %Give constraint lower bound
                ubg = [ubg;  0];          %Give constraint upper bound

            %           Hind Leg
                EqTemp = Ineq_Difference(FHx(k), Mfx, CH(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];       %Give constraint lower bound
                ubg = [ubg;  0];          %Give constraint upper bound
            %------------------------------------------------------           
            %       - Equation (2): Fx >= 0 - Mfx*C -->
            %                       Fx + Mfx*C >= 0 -->
            %                       0 <= Fx + Mfx*C
            %           Use Ineq_Summation
            %           Input: v[k] = F(F/H)x[k]
            %                  bigM = Mfx
            %                  z[k] = C(F/H)[k]
            %           lbg = 0
            %           ubg = inf
            %------------------------------------------------------
            %           Front Leg
                EqTemp = Ineq_Summation(FFx(k), Mfx, CF(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound

            %           Hind Leg
                EqTemp = Ineq_Summation(FHx(k), Mfx, CH(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound
            %------------------------------------------------------
            %     (-) y-axis force
            %------------------------------------------------------
            if TerrainType == 1 %Flat Terrain
            %       - Equation (1): Fy <= 0 + Mfy*C -->
            %                       Fy - Mfy*C <= 0
            %           Use Ineq_Difference
            %           Input: v[k] = F(F/H)y[k]
            %                  bigM = Mfy
            %                  z[k] = C(F/H)[k]
            %           lbg = -inf
            %           ubg = 0
            %           Front Leg
                EqTemp = Ineq_Difference(FFy(k), Mfy, CF(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];       %Give constraint lower bound
                ubg = [ubg;  0];          %Give constraint upper bound

            %           Hind Leg
                EqTemp = Ineq_Difference(FHy(k), Mfy, CH(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];          %Give constraint lower bound
                ubg = [ubg;  0];        %Give constraint upper bound
                
            %   Fy >= 0 Unilaterla constratin
                %           Front Leg
                lb_DecisionVars(find(VarNamesList == ['FFy_',num2str(k-1)])) = 0;
                %           Hind Leg
                lb_DecisionVars(find(VarNamesList == ['FHy_',num2str(k-1)])) = 0; 
                
            elseif TerrainType == 2 %Slope Terrain
                %   Fy <= 0 + Mfy*C --> Fy - Mfy*C <= 0
                %           Front Leg
                EqTemp = Ineq_Difference(FFy(k), Mfy, CF(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];       %Give constraint lower bound
                ubg = [ubg;  0];          %Give constraint upper bound

                %           Hind Leg
                EqTemp = Ineq_Difference(FHy(k), Mfy, CH(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  -inf];          %Give constraint lower bound
                ubg = [ubg;  0];        %Give constraint upper bound
                %   Fy >= -Mfy*C --> Fy + Mfy*C >= 0 
                %           Front Leg
                EqTemp = Ineq_Summation(FFy(k), Mfy, CF(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound

                %           Hind Leg
                EqTemp = Ineq_Summation(FHy(k), Mfy, CH(PhaseIdx));
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound
                %Terrain Norm/Unilateral Constraint
                %Fn >= 0 (Normal Force) larger than 0 --> [Fx;Fy]'*TerrainNorm >= 0
                %           Front Leg
                EqTemp = [FFx(k);FFy(k)]'*TerrainNorm;
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound
                %           Hind Leg
                EqTemp = [FHx(k);FHy(k)]'*TerrainNorm;
                g   = {g{:}, EqTemp};     %Append to constraint function list
                lbg = [lbg;  0];          %Give constraint lower bound
                ubg = [ubg;  inf];        %Give constraint upper bound
            end
            
            % Complementarity Constraints Built  
            %------------------------------------------------------
            
            %----------------------------------------------------
            % Friction Cone
            %----------------------------------------------------
            %   Reference: https://scaron.info/teaching/friction-cones.html
            %   General Equation:
            %   miu*TerrainNorm'*F + TerrainTangent'*F >= 0
            %   miu*TerrainNorm'*F - TerrainTangent'*F >= 0
            %
            %   Equation for 0-degree flat terrain: Fx[k] - miu*dot(Norm[k],Force[k]) <= 0
            %       Use Function FrictionCone
            %       Input: Norm[k]   = [Nx[k],Ny[k]]
            %              Force[k]  = [F(F/H)x[k], F(F/H)y[k]]
            %              Const_miu = miu
            %       lbg = -inf
            %       ubg = 0
            %----------------------------------------------------
            %   (Place Holder) Need to change terrain norm when move to (uneven)
            %   curvature terrain
            %----------------------------------------------------
              %       Front Leg
              EqTemp = miu*TerrainNorm'*[FFx(k);FFy(k)] + TerrainTangent'*[FFx(k);FFy(k)];
              g   = {g{:}, EqTemp};         %Append to constraint function list
              lbg = [lbg;  0];           %Give constraint lower bound
              ubg = [ubg;  inf];              %Give constraint upper bound
              
              EqTemp = miu*TerrainNorm'*[FFx(k);FFy(k)] - TerrainTangent'*[FFx(k);FFy(k)];
              g   = {g{:}, EqTemp};         %Append to constraint function list
              lbg = [lbg;  0];           %Give constraint lower bound
              ubg = [ubg;  inf];              %Give constraint upper bound
               
              %       Hind Leg
              EqTemp = miu*TerrainNorm'*[FHx(k);FHy(k)] + TerrainTangent'*[FHx(k);FHy(k)];
              g   = {g{:}, EqTemp};         %Append to constraint function list
              lbg = [lbg;  0];           %Give constraint lower bound
              ubg = [ubg;  inf];              %Give constraint upper bound
              
              EqTemp = miu*TerrainNorm'*[FHx(k);FHy(k)] - TerrainTangent'*[FHx(k);FHy(k)];
              g   = {g{:}, EqTemp};         %Append to constraint function list
              lbg = [lbg;  0];           %Give constraint lower bound
              ubg = [ubg;  inf];              %Give constraint upper bound


        %     %----------------------------------------------------
        %     % Torso y-axis level constraint (The Highest Bounding Box border should be always above the Ground)
        %     %----------------------------------------------------
        %     %   Equation: 1/2*BodyHeight <= y[k] - TerrainModel(x[k]) <= inf
        %     %   lbg = 0
        %     %   ubg = inf
        %         
        %         EqTemp = y(k) - TerrainModel(x(k));
        %         g   = {g{:}, EqTemp};
        %         lbg = [lbg; 1/2*BodyHeight];
        %         ubg = [ubg; inf];
        %     
            %----------------------------------------------------
            % Cost Function - Integral/Lagrangian Term
            %   Cost of Transport
            %J = J + h*FFx(k)^2 + h*FFy(k)^2 + h*FHx(k)^2 + h*FHy(k)^2 + h*PFxdot(k)^2 + h*PFydot(k)^2 + h*PHxdot(k)^2 + h*PHydot(k)^2;
            Scale_Factor = 1000; %difference below 1e-3 are treated as the same
            if cost_flag == 1 %Minimize Force Squared (Energy Loss)
                J = J + h*(FFx(k)^2) + h*(FFy(k)^2) + h*(FHx(k)^2) + h*(FHy(k)^2); 
            elseif cost_flag == 2 %Minimize Tangential Force (Maximize Robustness)
                %J = J + h*(FFx(k)^2) + h*(FHx(k)^2);
                J = J + h*((([FFx(k),FFy(k)]*TerrainTangent)*Scale_Factor)^2) + h*((([FHx(k),FHy(k)]*TerrainTangent)*Scale_Factor)^2);
            elseif cost_flag == 3 %Minimize Vibration (theta towards terrain slope, thetadot towards zero, ydot towards zero)
                if cost_type_flag == 1 %with Time Integral
                    J = J + h*((theta(k)-terrain_slope_rad)^2) + h*(thetadot(k)^2) + h*(([xdot(k),ydot(k)]*TerrainNorm)^2);
                elseif cost_type_flag == 2 %with Time Integral - Scaled
                    J = J + h*(((theta(k)-terrain_slope_rad)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2);
                elseif cost_type_flag == 3 %No Time Integral
                    J = J + (theta(k)-terrain_slope_rad)^2 + thetadot(k)^2 + ([xdot(k),ydot(k)]*TerrainNorm)^2;
                elseif cost_type_flag == 4 %No Time Integral - Scaled
                    J = J + ((theta(k)-terrain_slope_rad)*Scale_Factor)^2 + (thetadot(k)*Scale_Factor)^2 + (([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2;
                elseif cost_type_flag == 5 %infinity norm
                    J = norm((theta-terrain_slope_rad)*Scale_Factor,inf) + norm(thetadot*Scale_Factor,inf) + norm(([xdot,ydot]*TerrainNorm)*Scale_Factor,inf);
                end
            elseif cost_flag == 4 %5 -> Maximize Velocity Smoothness (x_tangent towards desired speed, ydot towards zero, thetadot towards zero)
                if cost_type_flag == 1 %with Time Integral
                    J = J + h*(([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2) + h*(([xdot(k),ydot(k)]*TerrainNorm)^2) + h*(thetadot(k)^2);
                elseif cost_type_flag == 2 %with Time Integral - Scaled
                    J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2);
                    %J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2);
                elseif cost_type_flag == 3 %No Time Integral
                    J = J + ([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2 + ([xdot(k),ydot(k)]*TerrainNorm)^2 + thetadot(k)^2;
                elseif cost_type_flag == 4 %No Time Integral - Scaled
                    J = J + (([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2 + (([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2 + (thetadot(k)*Scale_Factor)^2;                
                elseif cost_type_flag == 5 %infinity norm
                    J = norm(([xdot,ydot]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor,inf) + norm(([xdot,ydot]*TerrainNorm)*Scale_Factor,inf) + norm((thetadot*Scale_Factor),inf);
                end
            elseif cost_flag == 5 %Minimize Velocity Smoothnes with Fixed Orientatation (add orientation the same as the terrain slope)
                if cost_type_flag == 1 %with Time Integral
                    J = J + h*(([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2) + h*(([xdot(k),ydot(k)]*TerrainNorm)^2) + h*(thetadot(k)^2) + h*((theta(k)-terrain_slope_rad)^2);
                elseif cost_type_flag == 2 %with Time Integral - Scaled
                    J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2) + h*(((theta(k)-terrain_slope_rad)*Scale_Factor)^2);
                    %J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2);
                elseif cost_type_flag == 3 %No Time Integral
                    J = J + ([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2 + ([xdot(k),ydot(k)]*TerrainNorm)^2 + thetadot(k)^2 + (theta(k)-terrain_slope_rad)^2;
                elseif cost_type_flag == 4 %No Time Integral - Scaled
                    J = J + h*(([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2 + h*(([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2 + h*(thetadot(k)*Scale_Factor)^2 + h*((theta(k)-terrain_slope_rad)*Scale_Factor)^2;                
                elseif cost_type_flag == 5 %infinity norm
                    J = norm(([xdot,ydot]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor,inf) + norm(([xdot,ydot]*TerrainNorm)*Scale_Factor,inf) + norm(thetadot,inf) + norm((theta-terrain_slope_rad)*Scale_Factor,inf);
                end
            elseif cost_flag == 6 %Feet Velocity (Pending)
                J = J + h*(PFxdot(k)^2) + h*(PFydot(k)^2) + h*(PHxdot(k)^2) + h*(PHydot(k)^2);
            elseif cost_flag == 7 %Humanoid smooth motion
                J = J + h*((theta(k)*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2);
%             
%           elseif cost_flag == 4 %Minimize the speed difference between the desired speed at every knot (along the tangential line of the terrain)
%                 if cost_type_flag == 1 %with Time Integral
%                     J = J + h*([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2;
%                 elseif cost_type_flag == 2 %with Time Integral - Scaled
%                     J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2);
%                 elseif cost_type_flag == 3 %No Time Integral
%                     J = J + ([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2;
%                 elseif cost_type_flag == 4 %No Time Integral - Scaled
%                     J = J + (([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2;
%                 end    
%                 %Trapzoidal Integration
%                 %J = J + 1/2*h*((xdot(k+1)-speed)^2 + (xdot(k)-speed)^2) + 1/2*h*((ydot(k+1)-speed*tan(terrain_slope_rad))^2 + (ydot(k)-speed*tan(terrain_slope_rad))^2);
%                 %Euler Integration with Scale factor
%                 %J = J + h*(((xdot(k)-speed)*Scale_Factor)^2) + h*(((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2);
%                 %J = J + h*([xdot(k),ydot(k)]*TerrainTangent)^2;
%                 %No time
%                 %J = J + ((xdot(k)-speed)*Scale_Factor)^2 + ((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2;
%             elseif cost_flag == 4 %Minimize tangential speed (to the desired speed), normal axis speed (towards zero)
%                 if cost_type_flag == 1 %with Time Integral
%                     J = J + h*([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2 + h*([xdot(k),ydot(k)]*TerrainNorm)^2;
%                 elseif cost_type_flag == 2 %with Time Integral - Scaled
%                     J = J + h*((([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2);
%                 elseif cost_type_flag == 3 %No Time Integral
%                     J = J + ([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))^2 + ([xdot(k),ydot(k)]*TerrainNorm)^2;
%                 elseif cost_type_flag == 4 %No Time Integral - Scaled
%                     J = J + (([xdot(k),ydot(k)]*TerrainTangent - speed/cos(terrain_slope_rad))*Scale_Factor)^2 + (([xdot(k),ydot(k)]*TerrainNorm)*Scale_Factor)^2;
%                 end    
                %J = J + h*(xdot(k)-speed)^2 + h*(ydot(k)-speed*tan(terrain_slope_rad))^2 + h*([xdot(k);ydot(k)]'*TerrainNorm)^2;
                %Euler Integration
                %J = J + h*(((xdot(k)-speed)*Scale_Factor)^2) + h*(((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2) + h*(((ydot(k)/cos(terrain_slope_rad))*Scale_Factor)^2);
                %No Time
                %J = J + ((xdot(k)-speed)*Scale_Factor)^2 + ((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2 + ((ydot(k)/cos(terrain_slope_rad))*Scale_Factor)^2; 
                %J = J + h*(xdot(k)-speed)^2 + h*(ydot(k)-speed*tan(terrain_slope_rad))^2 + h*([xdot(k);ydot(k)]'*TerrainNorm)^2 + h*thetadot(k)^2;
                %Euler Integration
                %J = J + h*(((xdot(k)-speed)*Scale_Factor)^2) + h*(((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2) + h*(((ydot(k)/cos(terrain_slope_rad))*Scale_Factor)^2) + h*((thetadot(k)*Scale_Factor)^2);
                %No Time
                %J = J + ((xdot(k)-speed)*Scale_Factor)^2 + ((ydot(k)-speed*tan(terrain_slope_rad))*Scale_Factor)^2 + ((ydot(k)/cos(terrain_slope_rad))*Scale_Factor)^2 + (thetadot(k)*Scale_Factor)^2;
                %-----------------
            end
            %VelCostWweight = 500;
            %J = J + h*FFx(k)^2 + h*FFy(k)^2 + h*FHx(k)^2 + h*FHy(k)^2 + VelCostWweight*h*PFxdot(k)^2 + VelCostWweight*h*PFydot(k)^2 + VelCostWweight*h*PHxdot(k)^2 + VelCostWweight*h*PHydot(k)^2;
            %J = J + h*xdot(k)^2;%h*ydot(k)^2 + h*thetadot(k)^2;% + h*thetadot(k)^2; h*xdot(k)^2 + 
            %----------------------------------------------------
        end
        
% %!!!!!!!!!Add final knot in the cost, but should not add every time in the loop, put if k == NumKnots   
%         if k == tauSeriesLength
%             if cost_flag == 2 %Minimize Vibration
%                 J = J + h*(theta(end)-terrain_slope_rad)^2 + h*thetadot(end)^2 + h*([xdot(end);ydot(end)]'*TerrainNorm)^2;
%             elseif cost_flag == 3 %Minimize the speed difference between the desired speed at every knot (along the tangential line of the terrain)
%                 J = J + h*(xdot(end)-speed)^2 + h*(ydot(end)-speed*tan(terrain_slope_rad))^2;
%             elseif cost_flag == 4 %Minimize tangential speed (to the desired speed), normal axis speed (towards zero)
%                 J = J + h*(xdot(end)-speed)^2 + h*(ydot(end)-speed*tan(terrain_slope_rad))^2 + h*([xdot(end);ydot(end)]'*TerrainNorm)^2;
%             elseif cost_flag == 5 %5 -> Minimize tangential speed (to the desired speed), normal speed (tp zero), angular speed thetadot (to zero)
%                 J = J + h*(xdot(end)-speed)^2 + h*(ydot(end)-speed*tan(terrain_slope_rad))^2 + h*([xdot(end);ydot(end)]'*TerrainNorm)^2 + h*thetadot(end)^2;
%             end
%         end

        %----------------------------------------------------
        % Kinematics Constraint
        %----------------------------------------------------
        %   Equation: -[bw;bh]/2 <= R(theta[k])*(P[k] - [x[k];y[k]])-Pcenter <= [bw;bh]/2
        %       Use Function KinematicsConstraint 
        %       Input: r[k] = [x[k],y[k],theta[k]]         --> Robot Torso State
        %              Pk   = [P(F/H)x[k],P(F/H)y[k]]      --> Foot/End-Effector Location
        %              Pc   = [PcenterX, PcenterY]         --> Default Foot/End-Effector Location
        %       lbg = -[bw;bh]/2 
        %       ubg = [bw/bh]/2   --> bw is bounding box width, bh is boungding box height
        %----------------------------------------------------
        %     Front Leg
            EqTemp = KinematicsConstraint([x(k), y(k), theta(k)], [PFx(k), PFy(k)], [PFCenterX, PFCenterY]);
            g   = {g{:}, EqTemp};                                              %Append to constraint function list
            lbg = [lbg;  -[BoundingBox_Width;BoundingBox_Height]/2];           %Give constraint lower bound
            ubg = [ubg;  [BoundingBox_Width;BoundingBox_Height]/2];            %Give constraint upper bound

        %     Hind Leg
            EqTemp = KinematicsConstraint([x(k), y(k), theta(k)], [PHx(k), PHy(k)], [PHCenterX, PHCenterY]);
            g   = {g{:}, EqTemp};                                              %Append to constraint function list
            lbg = [lbg;  -[BoundingBox_Width;BoundingBox_Height]/2];           %Give constraint lower bound
            ubg = [ubg;  [BoundingBox_Width;BoundingBox_Height]/2];            %Give constraint upper bound
    end

    %-----------------------------------------------------------------------
    %   Switching Time Constraints
    %       Equations: 0 <= Ts[1] <= Ts[2] <= ... <= Ts[end]
    %       In CasAdi Form:
    %           (*) 0 <= Ts[1] - 0 (Init Time)
    %               Lower Bound: 0
    %               Upper Bound: inf
    %           (*) 0 <= Ts[2] - Ts[1] ... 0 <= Ts[i] - Ts[i-1]; for i from 2
    %           to NumPhase - 1
    %               Lower Bound: 0
    %               Upper Bound: inf
    %           (*) Ts[end] = Tend (Terminal Time Constraint)
    %               - Achieve by Setting up Variable Bounds (Lower Bound = Upper Bound = Tend)
    %-----------------------------------------------------------------------
    %       Phase Lower Bound
    %Phaselb = 0.1;
    %Phaselb = 0;
    %InitTerminalPhaselb = 0; %0.01
    phaselb = Tend*phase_lower_bound_portion
    for i = 1:NumPhases
        if i == 1
            EqTemp = Ts(1) - 0;
            g = {g{:}, EqTemp};
            lbg = [lbg; phaselb];
            ubg = [ubg; inf];
        elseif i == NumPhases
            EqTemp = Ts(i) - Ts(i-1);
            g = {g{:}, EqTemp};
            lbg = [lbg; phaselb];
            ubg = [ubg; inf]; 
        else
            EqTemp = Ts(i) - Ts(i-1);
            g = {g{:}, EqTemp};
            lbg = [lbg; phaselb];
            ubg = [ubg; inf]; 
        end
    end
    %-----------------------------------------
    %    Terminal Time Constraint
    %-----------------------------------------
    if Tend_flag == 1 %only constrain terminal time when user specifies that
        lb_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend;
        ub_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend;
    elseif Tend_flag == 2 %Bounded Terminal Time
        lb_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = 0.1;
        ub_DecisionVars(find(VarNamesList == ['Ts_',num2str(NumPhases)])) = Tend_Bound;
    end
    %-----------------------------------------------------------------------

    %-----------------------------------------------------------------------
    %   Periodical Motion Constraints
    %       Equation (Except X-axis positions): State_Init - State_End = 0
    %       lbg = 0
    %       ubg = 0
    %-----------------------------------------------------------------------
    %       X-axis positions
    %-----------------------------------------------------------------------
    %           Initial X-axis Position -> always = 0
    lb_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = 0;
    ub_DecisionVars(find(VarNamesList == ['x_',num2str(0)])) = 0;
    %           Terminal X-axis Position
    if Tend_flag == 1 %specified Tend
       lb_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = speed*Tend;
       ub_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = speed*Tend; 
    elseif Tend_flag == 2 %Bounded Terminal Time -> x_end = speed*Tend -> x_end - speed*Tend = 0
        EqTemp = x(end) - speed*Ts(end);
        g = {g{:}, EqTemp};
        lbg = [lbg; 0];
        ubg = [ubg; 0];
        %Minimum Locomotion Distrance for Terminal X-axis Position
        lb_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = BodyLength/2;
        ub_DecisionVars(find(VarNamesList == ['x_',num2str(NumKnots)])) = 5*BodyLength;
    end
    %-----------------------------------------------------------------------
    %       Y-axis Positions
    %-----------------------------------------------------------------------
    if TerrainType == 1 %Flat Terrain
        EqTemp = y(1) - y(end);
    elseif TerrainType == 2 %Slope Terrain
        EqTemp = y(end) - y(1) - speed*Tend*tan(terrain_slope_rad); 
    end
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %-----------------------------------------------------------------------
    %       Theta Positions
    %-----------------------------------------------------------------------
    EqTemp = theta(1) - theta(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %-----------------------------------------------------------------------
    %       X-axis Velocities
    %-----------------------------------------------------------------------
    EqTemp = xdot(1) - xdot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %-----------------------------------------------------------------------
    %       Y-axis Velocities
    %-----------------------------------------------------------------------
    EqTemp = ydot(1) - ydot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %-----------------------------------------------------------------------
    %       Angular Velocity
    %-----------------------------------------------------------------------
    EqTemp = thetadot(1) - thetadot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %-----------------------------------------------------------------------
    %   Front Leg Positions (Convert to Robot Local Frame)
    %-----------------------------------------------------------------------
    EqTemp = [cos(theta(1)), -sin(theta(1)); sin(theta(1)), cos(theta(1))]'*([PFx(1),PFy(1)]' - [x(1),y(1)]') - [cos(theta(end)), -sin(theta(end)); sin(theta(end)), cos(theta(end))]'*([PFx(end),PFy(end)]' - [x(end),y(end)]');
    g = {g{:}, EqTemp};
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
    %-----------------------------------------------------------------------
    %   Hind Leg Positions (Convert to Robot Local Frame)
    %-----------------------------------------------------------------------
    EqTemp = [cos(theta(1)), -sin(theta(1)); sin(theta(1)), cos(theta(1))]'*([PHx(1),PHy(1)]' - [x(1),y(1)]') - [cos(theta(end)), -sin(theta(end)); sin(theta(end)), cos(theta(end))]'*([PHx(end),PHy(end)]' - [x(end),y(end)]');
    g = {g{:}, EqTemp};
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
    % %-----------------------------------------------------------------------
    % %   Contact Configurations (Front Leg)
    % %-----------------------------------------------------------------------
    % EqTemp = CF(1) - CF(end);
    % g = {g{:}, EqTemp};
    % lbg = [lbg; 0];
    % ubg = [ubg; 0];
    % %-----------------------------------------------------------------------
    % %   Contact Configuration (Hind Leg)
    % %-----------------------------------------------------------------------
    % EqTemp = CH(1) - CH(end);
    % g = {g{:}, EqTemp};
    % lbg = [lbg; 0];
    % ubg = [ubg; 0];
    % %-----------------------------------------------------------------------
    %------------------------------------------------------------------------
    % Gait Constraint
    %------------------------------------------------------------------------
    FrontLeg_Contact = 0;
    HindLeg_Contact = 0;
    %for i = 2:NumPhases
    %    FrontLeg_Contact = FrontLeg_Contact + abs(CF(i)-CF(i-1))*if_else(Ts(i)-Ts(i-1)<=1e-2,0,1);
    %    HindLeg_Contact  = HindLeg_Contact  + abs(CH(i)-CH(i-1))*if_else(Ts(i)-Ts(i-1)<=1e-2,0,1);
    %end
    
    for i = 2:NumPhases
        FrontLeg_Contact = FrontLeg_Contact + abs(CF(i)-CF(i-1));
        HindLeg_Contact  = HindLeg_Contact  + abs(CH(i)-CH(i-1));
    end
    
    g = {g{:},FrontLeg_Contact};
    lbg = [lbg;0];
    ubg = [ubg;2];
    
    g = {g{:},HindLeg_Contact};
    lbg = [lbg;0];
    ubg = [ubg;2];
    
    disp('Constraints and Objetive Function Constructed')
    disp('===================================================')
    disp(' ')
    %=======================================================================

    %=======================================================================
    % Solve the Problem
    %=======================================================================
    %   Display some Info
    disp('===================================================')
    disp('Optimization Started')
    %   Assemble optimization problem definitions
    prob = struct('f', J, 'x', DecisionVars, 'g', vertcat(g{:}));

    %       Build Solver Option Structure
    if strcmp(SolverSelected, 'knitro')
        solverOption = struct('mip_outinterval', 100,...      % (Log Output Frequency) Log Output per Nodes
                              'mip_heuristic',   0,...     %-1,let sover to select heuristics method, 0, disable heuristics
                              'mip_outlevel',    2,...      % Print accumulated time for every node.
                              'mip_selectrule',  3,...      % 
                              'mip_branchrule',  2,...      % MIP Branching rule The rule for selecting nodes 2 has the best performance
                              'mip_maxnodes',    NumMaxNodes);      % Max Number of Nodes wish to be explored
  %                            'Multistart',      1,...      % Open multi start
  %                            'ms_maxsolves',    NumMultiStartSolves);      % Maximum CPU time
   %                           'par_numthreads',  2,...
   %                           'ms_deterministic',0);
   %                           'par_msnumthreads',3 ...


    elseif strcmp(SolverSelected, 'bonmin')
        solverOption = struct('option_file_name', 'bonmin.opt');  
    end

    %   Construct Nonlinear Programming Function Object
    solver = nlpsol('solver', SolverSelected, prob, struct('discrete', varstype, SolverSelected, solverOption));

    %   Solver the Problem
    sol = solver('x0',  DecisionVarsInit, ...
                 'lbx', lb_DecisionVars,...
                 'ubx', ub_DecisionVars,...
                 'lbg', lbg,...
                 'ubg', ubg);
    return_status = solver.stats();
    return_status.success 
    disp('===================================================')
    disp(' ')
    %=======================================================================
    % Extract the Solution and Visualization
    %=======================================================================
    %   Display some info
    disp('===================================================');
    disp('Result Extraction:');
    disp('---------------------------------------------------');
    %-----------------------------------------------------------------------
    %   Recover the full solution
    %-----------------------------------------------------------------------
    res = full(sol.x);

    %   Extract Switching Time
    PhaseSwitchingTime = [0;res(find(VarNamesList == Ts_label(1)):find(VarNamesList == Ts_label(end)))]; %include initial time 0
    TimeSlope = NumPhases.*diff(PhaseSwitchingTime);
    TimeIntercept = [PhaseSwitchingTime(1:end-1)];

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
    % Robot state (position)
    x_result     = res(find(VarNamesList == 'x_0'):find(VarNamesList == x_label(end)));
    y_result     = res(find(VarNamesList == 'y_0'):find(VarNamesList == y_label(end)));
    theta_result = res(find(VarNamesList == 'theta_0'):find(VarNamesList == theta_label(end)));

    % Robot state (Velocity)
    xdot_result = res(find(VarNamesList == 'xdot_0'):find(VarNamesList == xdot_label(end)));
    ydot_result = res(find(VarNamesList == 'ydot_0'):find(VarNamesList == ydot_label(end)));
    thetadot_result = res(find(VarNamesList == 'thetadot_0'):find(VarNamesList == thetadot_label(end)));

    % End-effector locations
    PFx_result = res(find(VarNamesList == 'PFx_0'):find(VarNamesList == PFx_label(end)));
    PFy_result = res(find(VarNamesList == 'PFy_0'):find(VarNamesList == PFy_label(end)));
    PHx_result = res(find(VarNamesList == 'PHx_0'):find(VarNamesList == PHx_label(end)));
    PHy_result = res(find(VarNamesList == 'PHy_0'):find(VarNamesList == PHy_label(end)));

    % End-effector velocities
    PFxdot_result = res(find(VarNamesList == 'PFxdot_0'):find(VarNamesList == PFxdot_label(end)));
    PFydot_result = res(find(VarNamesList == 'PFydot_0'):find(VarNamesList == PFydot_label(end)));
    PHxdot_result = res(find(VarNamesList == 'PHxdot_0'):find(VarNamesList == PHxdot_label(end)));
    PHydot_result = res(find(VarNamesList == 'PHydot_0'):find(VarNamesList == PHydot_label(end)));

    % Contact Configuration
    CF_result = res(find(VarNamesList == CF_label(1)):find(VarNamesList == CF_label(end)));
    CH_result = res(find(VarNamesList == CH_label(1)):find(VarNamesList == CH_label(end)));

    % Contact force result
    FFx_result = res(find(VarNamesList == 'FFx_0'):find(VarNamesList == FFx_label(end)));
    FFy_result = res(find(VarNamesList == 'FFy_0'):find(VarNamesList == FFy_label(end)));
    FHx_result = res(find(VarNamesList == 'FHx_0'):find(VarNamesList == FHx_label(end)));
    FHy_result = res(find(VarNamesList == 'FHy_0'):find(VarNamesList == FHy_label(end)));

    NetForceX = FFx_result + FHx_result;
    NetForceY = FFy_result + FHy_result;

    % Torque on the body
    FrontTorque_result = (PFx_result(1:end-1) - x_result(1:end-1)).*FFy_result - (PFy_result(1:end-1) - y_result(1:end-1)).*FFx_result;
    HindTorque_result  = (PHx_result(1:end-1) - x_result(1:end-1)).*FHy_result - (PHy_result(1:end-1) - y_result(1:end-1)).*FHx_result;

    NetTorque = FrontTorque_result + HindTorque_result;

    % Foot Bounding Box Result
    PFcenterX_result_world = x_result + cos(theta_result)*PFCenterX - sin(theta_result)*PFCenterY;
    PFcenterY_result_world = y_result + sin(theta_result)*PFCenterX + cos(theta_result)*PFCenterY;

    PHcenterX_result_world = x_result + cos(theta_result)*PHCenterX - sin(theta_result)*PHCenterY;
    PHcenterY_result_world = y_result + sin(theta_result)*PHCenterX + cos(theta_result)*PHCenterY;

    disp('Result Variables Extracted')
    %---------------------------------------------------------------------
    % Backup Original Results
    %---------------------------------------------------------------------
    TimeSeries_origin  = TimeSeries;
    x_result_origin    = x_result;       y_result_origin    = y_result;      theta_result_origin    = theta_result;
    xdot_result_origin = xdot_result;    ydot_result_origin = ydot_result;   thetadot_result_origin = thetadot_result;
    PFx_result_origin  = PFx_result;     PFy_result_origin  = PFy_result;    PFxdot_result_origin   = PFxdot_result;      PFydot_result_origin = PFydot_result;
    PHx_result_origin  = PHx_result;     PHy_result_origin  = PHy_result;    PHxdot_result_origin   = PHxdot_result;      PHydot_result_origin = PHydot_result;
    CF_result_origin   = CF_result;      CH_result_origin   = CH_result;
    FFx_result_origin  = FFx_result;     FFy_result_origin  = FFy_result;    
    FHx_result_origin  = FHx_result;     FHy_result_origin  = FHy_result;
    NetForceX_origin   = NetForceX;      NetForceY_origin   = NetForceY;
    FrontTorque_result_origin = FrontTorque_result;      HindTorque_result_origin = HindTorque_result;
    NetTorque_origin   = NetTorque;
    PFcenterX_result_world_origin = PFcenterX_result_world;      PFcenterY_result_world_origin = PFcenterY_result_world;
    PHcenterX_result_world_origin = PHcenterX_result_world;      PHcenterY_result_world_origin = PHcenterY_result_world;
    disp('Original Result Variables Backuped - Result Include Vanishing Phases - End with "origin"');
    %-----------------------------------------------------------------------

    %-----------------------------------------------------------------------
    % Clean Up Time/Control/State Lists - Remove Phases with Zero Length  
    %-----------------------------------------------------------------------
    TimeStepDiff = diff(TimeSeries);

    %States, TimeStepDiff + 1
    TimeSeries(find(TimeStepDiff <= 1e-3) + 1) = [];
    x_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    y_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    theta_result(find(TimeStepDiff <= 1e-3) + 1) = [];

    xdot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    ydot_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    thetadot_result(find(TimeStepDiff <= 1e-3) + 1) = [];

    PFcenterX_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
    PFcenterY_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

    PHcenterX_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];
    PHcenterY_result_world(find(TimeStepDiff <= 1e-3) + 1) = [];

    %Feet State, TimeStepDiff due to euler integration
    PFx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    PFy_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    PHx_result(find(TimeStepDiff <= 1e-3) + 1) = [];
    PHy_result(find(TimeStepDiff <= 1e-3) + 1) = [];

    %Input

    FFx_result(find(TimeStepDiff <= 1e-3)) = [];
    FFy_result(find(TimeStepDiff <= 1e-3)) = [];
    FHx_result(find(TimeStepDiff <= 1e-3)) = [];
    FHy_result(find(TimeStepDiff <= 1e-3)) = [];

    PFxdot_result(find(TimeStepDiff <= 1e-3)) = [];
    PFydot_result(find(TimeStepDiff <= 1e-3)) = [];
    PHxdot_result(find(TimeStepDiff <= 1e-3)) = [];
    PHydot_result(find(TimeStepDiff <= 1e-3)) = [];

    %Net Forces
    NetForceX(find(TimeStepDiff <= 1e-3)) = [];
    NetForceY(find(TimeStepDiff <= 1e-3)) = [];

    NetTorque(find(TimeStepDiff <= 1e-3)) = [];

    % NetForceX(end) = NetForceX(end - 1);
    % NetForceY(end) = NetForceY(end - 1);
    % NetTorque(end) = NetTorque(end - 1);

    %Clear the Input at last time step, for simulation
    NetForceX(end) = 0;
    NetForceY(end) = 0;
    NetTorque(end) = 0;

    %Phase Lengths

    PhaseLengths_origin = diff(PhaseSwitchingTime);

    PhaseLengths = PhaseLengths_origin;
    PhaseLengths(find(PhaseLengths_origin <= 1e-3 )) = [];

    %Contact Configurations
    CF_result(find(PhaseLengths_origin <= 1e-3 )) = [];
    CH_result(find(PhaseLengths_origin <= 1e-3 )) = [];

    gait = [CF_result, CH_result, PhaseLengths]

    disp('Removed Variables within Vanished Phases');
    disp('===================================================');
    disp(' ');
    %=======================================================================

    %recover some important info

    result_cost = full(sol.f); %cost function

    %=======================================================================
    % Close Diary
    diary off
    
    % Save Experimental Result
    warning('off')
    save([ExpDirectory, '/', ExpLog_filename, '.mat']);
    warning('on')

end

end