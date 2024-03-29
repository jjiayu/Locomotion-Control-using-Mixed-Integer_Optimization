% Mixed-Integer Nonlinear Optimization for 1D Ponit Mass Moving (Traslational Dynamics for Half-Cheetah)
% Mode Selection described as foot-ground contact changes 0/1

clear variables
% clc;

%==========================================================
%Inertia Parameters (Information from MIT Cheetah)
m = 1%33; %kg
I = 2.9; %kg m^2
g = 9.80665; %m/s^2
%==========================================================

%==========================================================
%Time parameters
h = 0.01; %Time Step in seconds
EndTime = h*20; %in seconds BONMIN can handle maximumly 30 time steps
% missing checking the endtime is the multiple of the time step
TimeSeries = 0:h:EndTime;
TimeSeriesLength = length(TimeSeries);
%=========================================================

%=========================================================
%Initial Conditions
x_init = 0;
xdot_init = 0;
%---------------------------------------------------------
%Terminal Conditions
x_end = 0.1;
xdot_end = 0;
%=========================================================

%==========================================================
%Define/Create Modeling Variables
%----------------------------------------------------------
%   States: r = [x,y,theta,xdot,ydot,thetadot]
%       x: horizontal position
%       y: vertical position
%       xdot: horizontal velocity
%       ydot: vertical velocity
%----------------------------------------------------------
%   Initialize Variable Name Containers
x_label = strings(1,TimeSeriesLength);
xdot_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    x_label(i) = strcat('x',num2str((i-1)));
    xdot_label(i) = strcat('xdot',num2str((i-1)));
end
%   Save List Length
xLength = length(x_label);
xdotLength = length(xdot_label);
%----------------------------------------------------------
%   Foot-Ground Reaction Forces(IN WORLD FRAME):
%       Front Leg Forces: FF = [FFx, FFy]
%       Hind Leg Forces:  FH = [FHx, FHy]
%----------------------------------------------------------
%   Initialize Variable Name Containers
FFx_label = strings(1,TimeSeriesLength);
FHx_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    FFx_label(i) = strcat('FFx',num2str((i-1)));
    FHx_label(i) = strcat('FHx',num2str((i-1))); 
end
%   Save List Length
FFxLength = length(FFx_label);
FHxLength = length(FHx_label);
%----------------------------------------------------------
%   Mode Selection (Defined as the contact configuration for each leg):
%       Leg Contact Configuration: C = [CF, CH]; CF,CH = 0/1 (Binary Varibale)
%           Front Leg Contact On/Off: CF
%           Hind Leg Contact On/Off:  CH
%   Initialize Variable Name Containers
CF_label = strings(1,TimeSeriesLength);
CH_label = strings(1,TimeSeriesLength);
%   Assign Variable Names
for i = 1:TimeSeriesLength
    CF_label(i) = strcat('CF',num2str((i-1)));
    CH_label(i) = strcat('CH',num2str((i-1)));
end
%   Save List Length
CFLength = length(CF_label);
CHLength = length(CH_label);
%----------------------------------------------------------
%   Full Decision Variable Name List
%Original names length
%names = [x_label, y_label, xdot_label, ydot_label, PFx_label, PFy_label, PFxdot_label, PFydot_label, PHx_label, PHy_label, PHxdot_label, PHydot_label, FFx_label, FFy_label, FHx_label, FHy_label, CF_label, CH_label];
names = [x_label, xdot_label, FFx_label, FHx_label, CF_label, CH_label];

%   Length of the name list
namesLength = length(names);

%   Length list of all variable list
%Original Length list
%LengthList = [xLength, yLength, xdotLength, ydotLength, PFxLength, PFyLength, PFxdotLength, PFydotLength, PHxLength, PHyLength, PHxdotLength, PHydotLength, FFxLength, FFyLength, FHxLength, FHyLength, CFLength, CHLength];
LengthList = [xLength, xdotLength, FFxLength, FHxLength, CFLength, CHLength];

%----------------------------------------------------------
%   List for all decision variable names
%varList = ["x", "y", "xdot", "ydot", "PFx", "PFy", "PFxdot", "PFydot", "PHx", "PHy", "PHxdot", "PHydot", "FFx", "FFy", "FHx", "FHy", "CF", "CH"];
varList = ["x", "xdot", "FFx", "FHx", "CF", "CH"];

%   Length of the decision varibale name list
varListLength = length(varList);
%==========================================================

%==========================================================
%Setup Optimization Problem
%----------------------------------------------------------
%   Objective Function
%       minimize F^2 -> FFx(t)^2 + FFy(t)^2 + FHx(t)^2 + FHy(t)^2
%   Quadratic form:
%       minimize F'*Q*F
%   v = [x;   y;   xdot;   ydot;   PFx;   PFy;  PFxdot;  PFydot;  PHx;  PHy;  PHxdot;  PHydot;   FFx;    FFy;    FHx;    FHy;   CF;   CH]
%   idx: 1    2     3        4      5      6      7         8      9     10     11       12       13      14      15     16     17    18
%   v'*Q*v = sigma(i,j) v1*Q11*v1 + v1*Q12*v2 + .... + v2*Q21*v1 + v2*Q22*v2 ...
%-----------------------------------------------------------
%   Define Q
QCell = {};
%   Assign Q Matrix, True Code need to be activated when running optimization
for i = 1:varListLength
    for j = 1:varListLength
        %Original cost coefficients
        %(varList(i) == "FFx" && varList{j} == "FFx") || (varList{i} == "FFy" && varList{j} == "FFy") || (varList{i} == "FHx" && varList{j} == "FHx") || (varList{i} == "FHy" && varList{j} == "FHy")
        if (varList(i) == "FFx" && varList{j} == "FFx") || (varList{i} == "FHx" && varList{j} == "FHx")
            QCell{i,j} = eye(LengthList(i),LengthList(j));
        else
            QCell{i,j} = zeros(LengthList(i),LengthList(j));
        end
    end
end

% % Test Code to assign Q matrix
% %Add integer variable into cost function
% for i = 1:varListLength
%     for j = 1:varListLength
%         if (varList(i) == "FFx" && varList{j} == "FFx") || (varList{i} == "FFy" && varList{j} == "FFy") || (varList{i} == "FHx" && varList{j} == "FHx") || (varList{i} == "FHy" && varList{j} == "FHy") || (varList{i} == "CF" && varList{j} == "CF") || (varList{i} == "CH" && varList{j} == "CH")
%             QCell{i,j} = eye(LengthList(i),LengthList(j));
%         else
%             QCell{i,j} = zeros(LengthList(i),LengthList(j));
%         end
%     end
% end

Q = sparse(cell2mat(QCell)); %sparse matrix may help

% Test, all variable are in cost function
%Q = eye(namesLength,namesLength);
%------------------------------------------------------------
%   Build Objective Function
%objfunc = @(v) v'*Q*v;
objfunc = @(vars)cost(vars,Q); %!!!!Cost function may need quadrature as well
%------------------------------------------------------------
%   Add Constraints
%       Torso Position Dynamics (equality constraints)
%-------------------------------------------------------------
%           x-axis position dynamics
%               Build A matrix
Ax_pos_dyn = zeros(xLength-1,namesLength); 
for k = 0:TimeSeriesLength-2 %NOTE: minus 2, 0 to (length -1) -1 = length -1
    Ax_pos_dyn(k+1,find(names == strcat('x',num2str(k+1)))) = 1;
    Ax_pos_dyn(k+1,find(names == strcat('x',num2str(k))))   = -1;
    Ax_pos_dyn(k+1,find(names == strcat('xdot',num2str(k+1))))= -1/2*h;
    Ax_pos_dyn(k+1,find(names == strcat('xdot',num2str(k))))= -1/2*h;
end
%               Build b vector
bx_pos_dyn = zeros(size(Ax_pos_dyn,1),1);
%---------------------------------------------------------------
%           Collect Position Dynamic Constraints
%---------------------------------------------------------------
Apos_dyn = [Ax_pos_dyn;];
    
bpos_dyn = [bx_pos_dyn;];
%---------------------------------------------------------------
%       Complementarity Constraints
%---------------------------------------------------------------
%           Parameter settings
%---------------------------------------------------------------
height = 0; %terrain height
Mpos_y = 1000; %big-M for Foot position in y-axis
Mvel = 1000; %big-M for Foot velocity in both x and y axis
Mfx = 1000; %big-M for foot-ground reaction forces for x-axis
Mfy = 1000; %big-M for foot-ground reaction forces for y-axis
%---------------------------------------------------------------
%           Front Leg
%---------------------------------------------------------------

% %---------------------------------------------------------------
%               Foot-Ground Reaction Forces
%---------------------------------------------------------------
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: FFx <= 0 + Mfx*CF -> FFx - Mfx*CF <= 0
%                       Build A matrix
AFFx_Con1 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFx_Con1(k,find(names == strcat('FFx',num2str(k-1)))) = 1;
    AFFx_Con1(k,find(names == strcat('CF',num2str(k-1))))  = -Mfx;
end
%                       Build b vector
bFFx_Con1 = zeros(size(AFFx_Con1,1),1);
%                       Setup Constraint Type
TypeFFx_Con1 = repmat(-1,size(AFFx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FFx >= 0 - Mfx*Cf -> FFx + Mfx*CF >= 0
%                       Build A matrix
AFFx_Con2 = zeros(CFLength,namesLength);
for k = 1:CFLength
    AFFx_Con2(k,find(names == strcat('FFx',num2str(k-1)))) = 1;
    AFFx_Con2(k,find(names == strcat('CF',num2str(k-1))))  = Mfx;
end
%                       Build b vector
bFFx_Con2 = zeros(size(AFFx_Con2,1),1);
%                       Setup Constraint Type
TypeFFx_Con2 = repmat(1,size(AFFx_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%               Foot-Ground Reaction Forces
%---------------------------------------------------------------
%                   x-axis
%---------------------------------------------------------------
%                       1st Constraint: FHx <= 0 + Mfx*CH -> FHx - Mfx*CH <= 0
%                       Build A matrix
AFHx_Con1 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHx_Con1(k,find(names == strcat('FHx',num2str(k-1)))) = 1;
    AFHx_Con1(k,find(names == strcat('CH',num2str(k-1))))  = -Mfx;
end
%                       Build b vector
bFHx_Con1 = zeros(size(AFHx_Con1,1),1);
%                       Setup Constraint Type
TypeFHx_Con1 = repmat(-1,size(AFHx_Con1,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%                       2nd Constraint: FHx >= 0 - Mfx*CH -> FHx + Mfx*CH >= 0
%                       Build A matrix
AFHx_Con2 = zeros(CHLength,namesLength);
for k = 1:CHLength
    AFHx_Con2(k,find(names == strcat('FHx',num2str(k-1)))) = 1;
    AFHx_Con2(k,find(names == strcat('CH',num2str(k-1))))  = Mfx;
end
%                       Build b vector
bFHx_Con2 = zeros(size(AFHx_Con2,1),1);
%                       Setup Constraint Type
TypeFHx_Con2 = repmat(1,size(AFHx_Con2,1),1);%-1 <=, 0 ==, 1 >=
%---------------------------------------------------------------
%           Collect Complementarity Constraints
Acomplementarity = [AFFx_Con1;AFFx_Con2;...
                    AFHx_Con1;AFHx_Con2;...
                    ];

bcomplementarity = [bFFx_Con1;bFFx_Con2;...
                    bFHx_Con1;bFHx_Con2;...
                    ];

Typecomplementarity = [TypeFFx_Con1;TypeFFx_Con2;...
                       TypeFHx_Con1;TypeFHx_Con2;...
                       ];
                
%---------------------------------------------------------------
%   Set Boundary Conditions
%---------------------------------------------------------------
%       Initial State
%           Build A matrix
Ax_init = zeros(1,namesLength);
Ax_init(find(names == 'x0')) = 1;
Axdot_init = zeros(1,namesLength);
Axdot_init(find(names == 'xdot0')) = 1;
%               Collect into an A matrix
Ainit = [Ax_init;
         Axdot_init;
        ];
%           Build b vectors
binit = [x_init;
         xdot_init;
         ];
%---------------------------------------------------------------
%       Terminal State
%           Build A matrix
Ax_end = zeros(1,namesLength);
Ax_end(find(names == x_label(end))) = 1;
Axdot_end = zeros(1,namesLength);
Axdot_end(find(names == xdot_label(end))) = 1;
%               Collect in to an A matrix
Aend = [Ax_end;
        Axdot_end;
        ];
%           Build b vectors
bend = [x_end;
        xdot_end;
        ];
%---------------------------------------------------------------
%===============================================================
%Build Constraints
%===============================================================
%   Collect all Aeq matrices and beq vectors for linear equality
%   constraints
Aeq = [Ainit;Aend;Apos_dyn]; %SPARSE?!!
beq = [binit;bend;bpos_dyn]; %SPARSE?!!
%---------------------------------------------------------------
%   Collect all A, b and Type for linear inequality constraints
A = [Acomplementarity];
b = [bcomplementarity];
e = [Typecomplementarity];
%---------------------------------------------------------------
%Nonlinear Constraints
%   Define Nonlinear Constraints Function Handle
nlcon = @(vars) nlconstraint(vars,TimeSeriesLength,names,m,h);
%   Right-hand side
x_dyn_rhs = zeros(TimeSeriesLength-1,1);   
%y_dyn_rhs = -h*g*ones(TimeSeriesLength-1,1);
nlcon_rhs = [x_dyn_rhs;];
%   nonlinear constraint type
nlcon_type = repmat(0,length(nlcon_rhs),1);

%------------------------------------------------------------
%   Variable Lower and Upper Boundaries, Inf slow down computing speed,
%   seems like
lb = repmat(-inf,1,namesLength);
ub = inf(1,namesLength);
%   Variable Type
vtype = [repmat('C', 1, xLength + xdotLength + FFxLength + FHxLength), repmat('B',1, CFLength + CHLength)];

%=============================================================

%=============================================================
%Solve Optimization Problem
%-------------------------------------------------------------
%   Initial Seed
v0 = ones(namesLength,1);
%   Optimizer Settings
opts = optiset('solver','bonmin','display','iter');
%   Run Optimization
% Opt = opti('fun',objfunc, 'eq',Aeq,beq, 'mix',A,b,e, 'nlmix',nlcon,nlcon_rhs,nlcon_type, 'bounds',lb,ub, 'xtype',vtype, 'options',opts);
Opt = opti('fun',objfunc, 'eq',Aeq,beq, 'nlmix',nlcon,nlcon_rhs,nlcon_type, 'bounds',lb,ub, 'xtype',vtype, 'options',opts);
%   Solve the Optimization Problem
[results,fval,exitflag,info] = solve(Opt,v0) 

% %Use Gurobi
% model.Q = sparse(cell2mat(QCell));
% model.modelsense = 'min';
% model.vtype = vtype;
% model.lb = lb;
% model.A = sparse([A;Aeq]);
% model.rhs = [b;beq];
% model.sense = e;
% model.sense(find(model.sense == -1)) ='<';
% model.sense(find(model.sense == 1)) ='>';
% model.sense = [model.sense;repmat('=',length(beq),1)];
% 
% result = gurobi(model);