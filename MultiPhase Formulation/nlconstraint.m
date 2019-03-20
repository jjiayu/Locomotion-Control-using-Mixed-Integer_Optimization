function [c,ceq] = nlconstraint(vars,...
                                NumPhases,...
                                NumLocalKnots,...
                                tauStepLength,...
                                m,...
                                g,...
                                I,...
                                xIdx_init,xIdx_end,...
                                yIdx_init,yIdx_end,...
                                thetaIdx_init,thetaIdx_end,...
                                xdotIdx_init,xdotIdx_end,...
                                ydotIdx_init,ydotIdx_end,...
                                thetadotIdx_init,thetadotIdx_end,...
                                PFxIdx_init,PFxIdx_end,...
                                PFyIdx_init,PFyIdx_end,...
                                PHxIdx_init,PHxIdx_end,...
                                PHyIdx_init,PHyIdx_end,...
                                PFxdotIdx_init,PFxdotIdx_end,...
                                PFydotIdx_init,PFydotIdx_end,...
                                PHxdotIdx_init,PHxdotIdx_end,...
                                PHydotIdx_init,PHydotIdx_end,...
                                FFxIdx_init,FFxIdx_end,...
                                FFyIdx_init,FFyIdx_end,...
                                FHxIdx_init,FHxIdx_end,...
                                FHyIdx_init,FHyIdx_end,...
                                SwitchingTimeIdx_init,SwitchingTimeIdx_end,...
                                PFcenterX,PFcenterY,...
                                PHcenterX,PHcenterY,...
                                BoundingBox_Width,BoundingBox_Height,...
                                TerrainNorm,miu)
%   OUTPUT
%       c: nonlinear inequality constraint
%       ceq: nonlinear equality constraint

%=========================================================================
%   Extract state, control vectors
%-------------------------------------------------------------------------
%   "Current" Vectors: start from t = 0 to t= end - 1, represent the state
%   and controls at t = k
%   These knots obey kinematics contraint and friction cone
x_current = vars(xIdx_init: xIdx_end - 1);
y_current = vars(yIdx_init: yIdx_end - 1);
theta_current = vars(thetaIdx_init: thetaIdx_end - 1);
xdot_current = vars(xdotIdx_init:xdotIdx_end - 1);
ydot_current = vars(ydotIdx_init:ydotIdx_end - 1);
thetadot_current = vars(thetadotIdx_init:thetadotIdx_end - 1);
PFx_current = vars(PFxIdx_init:PFxIdx_end - 1);
PFy_current = vars(PFyIdx_init:PFyIdx_end - 1);
PHx_current = vars(PHxIdx_init:PHxIdx_end - 1);
PHy_current = vars(PHyIdx_init:PHyIdx_end - 1);
PFxdot_current = vars(PFxdotIdx_init:PFxdotIdx_end - 1);
PFydot_current = vars(PFydotIdx_init:PFydotIdx_end - 1);
PHxdot_current = vars(PHxdotIdx_init:PHxdotIdx_end - 1);
PHydot_current = vars(PHydotIdx_init:PHydotIdx_end - 1);
FFx_current = vars(FFxIdx_init:FFxIdx_end - 1);
FFy_current = vars(FFyIdx_init:FFyIdx_end - 1);
FHx_current = vars(FHxIdx_init:FHxIdx_end - 1);
FHy_current = vars(FHyIdx_init:FHyIdx_end - 1);

%   "Future" Vectors: start from t = 1 to t = end, represnet the state and
%   control resultant from system dynamics t = k + 1
x_future = vars(xIdx_init + 1:xIdx_end);
y_future = vars(yIdx_init + 1:yIdx_end);
theta_future = vars(thetaIdx_init + 1:thetaIdx_end);
xdot_future = vars(xdotIdx_init + 1:xdotIdx_end);
ydot_future = vars(ydotIdx_init + 1:ydotIdx_end);
thetadot_future = vars(thetadotIdx_init + 1:thetadotIdx_end);
PFx_future = vars(PFxIdx_init + 1:PFxIdx_end);
PFy_future = vars(PFyIdx_init + 1:PFyIdx_end);
PHx_future = vars(PHxIdx_init + 1:PHxIdx_end);
PHy_future = vars(PHyIdx_init + 1:PHyIdx_end);
FFx_future = vars(FFxIdx_init + 1:FFxIdx_end);
FFy_future = vars(FFyIdx_init + 1:FFyIdx_end);
FHx_future = vars(FHxIdx_init + 1:FHxIdx_end);
FHy_future = vars(FHyIdx_init + 1:FHyIdx_end);

%Full Vectors
x_full = vars(xIdx_init: xIdx_end);
y_full = vars(yIdx_init: yIdx_end);
theta_full = vars(thetaIdx_init: thetaIdx_end);
xdot_full = vars(xdotIdx_init:xdotIdx_end);
ydot_full = vars(ydotIdx_init:ydotIdx_end);
thetadot_full = vars(thetadotIdx_init:thetadotIdx_end);
PFx_full = vars(PFxIdx_init:PFxIdx_end);
PFy_full = vars(PFyIdx_init:PFyIdx_end);
PHx_full = vars(PHxIdx_init:PHxIdx_end);
PHy_full = vars(PHyIdx_init:PHyIdx_end);

%   Time Step Vector
SwitchingTimeVector = vars(SwitchingTimeIdx_init:SwitchingTimeIdx_end); %Extract Switching Time Vector
%KnotIdxVector = 0:NumKnots-1; %Knot index vector, the last knot does not account for integration, ignore
SlopeVector = NumPhases.*diff([0;SwitchingTimeVector]);%Slope to 
TimeStep_vector = repelem(tauStepLength.*SlopeVector,NumLocalKnots); %time step vector in real-time scale

%   Old vector definition
% x = vars(xIdx_init:xIdx_end-1);
% y = vars(yIdx_init:yIdx_end-1);
% theta = vars(thetaIdx_init:thetaIdx_end-1);
% thetadot_future = vars(thetadotIdx_init+1:thetadotIdx_end); %tehtadot k+1
% thetadot_current = vars(thetadotIdx_init:thetadotIdx_end-1);%thetadot k
% PFx = vars(PFxIdx_init:PFxIdx_end-1);
% PFy = vars(PFyIdx_init:PFyIdx_end-1);
% PHx = vars(PHxIdx_init:PHxIdx_end-1);
% PHy = vars(PHyIdx_init:PHyIdx_end-1);
% FFx = vars(FFxIdx_init:FFxIdx_end-1);
% FFy = vars(FFyIdx_init:FFyIdx_end-1);
% FHx = vars(FHxIdx_init:FHxIdx_end-1);
% FHy = vars(FHyIdx_init:FHyIdx_end-1);

%=========================================================================
%       System Dynamics
%=========================================================================
%       Robot Torso Dynamics
%-------------------------------------------------------------------------
%           x-axis position dynamics
x_pos_dyn = x_future - x_current - TimeStep_vector.*xdot_current;
%-------------------------------------------------------------------------
%           x-axis velocity dynamics
x_vel_dyn = xdot_future - xdot_current - TimeStep_vector./m.*FFx_current - TimeStep_vector./m.*FHx_current;
%-------------------------------------------------------------------------
%           y-axis position dynamics
y_pos_dyn = y_future - y_current - TimeStep_vector.*ydot_current;
%-------------------------------------------------------------------------
%           y-axis velocity dynamics
y_vel_dyn = ydot_future - ydot_current - TimeStep_vector./m.*FFy_current - TimeStep_vector./m.*FHy_current + TimeStep_vector.*g;
%-------------------------------------------------------------------------
%           theta position dynamics
theta_pos_dyn = theta_future - theta_current - TimeStep_vector.*thetadot_current;
%-------------------------------------------------------------------------
%           theta velocity dynamics
%           Rotation dynamics --> I*thetadotodt = (PF - r) X FF + (PH - r) X FH
%-------------------------------------------------------------------------
%           Build constraint
torque = (PFx_current - x_current).*FFy_current - (PFy_current - y_current).*FFx_current + (PHx_current - x_current).*FHy_current - (PHy_current - y_current).*FHx_current;
theta_vel_dyn = I.*thetadot_future - I.*thetadot_current - TimeStep_vector.*torque;
%-------------------------------------------------------------------------
%       Foot/End-Effector Dynamics
%-------------------------------------------------------------------------
%           Front Leg x-axis position dynamics
%-------------------------------------------------------------------------
PFx_pos_dyn = PFx_future - PFx_current - TimeStep_vector.*PFxdot_current;
%-------------------------------------------------------------------------
%           Front Leg y-axis position dynamics
PFy_pos_dyn = PFy_future - PFy_current - TimeStep_vector.*PFydot_current;
%-------------------------------------------------------------------------
%           Hind Leg x-axis position dynamics
PHx_pos_dyn = PHx_future - PHx_current - TimeStep_vector.*PHxdot_current;
%-------------------------------------------------------------------------
%           Hind Leg y-axis position dynamics
PHy_pos_dyn = PHy_future - PHy_current - TimeStep_vector.*PHydot_current;
%=========================================================================


%=========================================================================
%       Kinematics Constraint
%           Move to tensor-based formulation to speed up computation and
%           problem formulation --> Ask Chris and Theo
%=========================================================================
%       R = [cos(theta), -sin(theta);
%            sin(theta), cos(theta)]
%************************************************************
%       R^T (rotation from world frame to local frame) = 
%         = [cos(theta), sin(theta)
%            -sin(theta),cos(theta)]
%************************************************************
%-------------------------------------------------------------------------
%       Front Leg:
PFx_RobotFrame = cos(theta_full).*(PFx_full - x_full) + sin(theta_full).*(PFy_full - y_full);
PFy_RobotFrame = -sin(theta_full).*(PFx_full - x_full) + cos(theta_full).*(PFy_full - y_full);
%           Kinematics Constraint 1: 
%           PFx_RobotFrame - PFcenterX <= BoundingBox_Width/2 --> 
%           PFx_RobotFrame - PFcenterX - BoundingBox_Width/2 <= 0
FrontLegKine1 = PFx_RobotFrame - PFcenterX - BoundingBox_Width/2;
%           Kinematics Constraint 2:
%           PFy_RobotFrame - PFcenterY <= BoundingBox_Height/2 -->
%           PFy_RobotFrame - PFcenterY - BoundingBox_Height/2 <= 0
FrontLegKine2 = PFy_RobotFrame - PFcenterY - BoundingBox_Height/2;
%           Kinematics Constraint 3:
%           PFx_RobotFrame - PFcenterX >= -BoundingBox_Width/2 -->
%           -PFx_RobotFrame + PFcenterX - BoundingBox_Width/2 <= 0
FrontLegKine3 = -PFx_RobotFrame + PFcenterX - BoundingBox_Width/2;
%           Kinematics Constraint 4:
%           PFy_RobotFrame - PFcenterY >= -BoundingBox_Height/2 -->
%           -PFy_RobotFrame + PFcenterY - BoundingBox_Height/2 <= 0
FrontLegKine4 = -PFy_RobotFrame + PFcenterY - BoundingBox_Height/2;
%-------------------------------------------------------------------------
%       Hind Leg:
PHx_RobotFrame = cos(theta_full).*(PHx_full - x_full) + sin(theta_full).*(PHy_full - y_full);
PHy_RobotFrame = -sin(theta_full).*(PHx_full - x_full) + cos(theta_full).*(PHy_full - y_full);
%           Kinematics Constraint 1:
%           PHx_RobotFrame - PHcenterX <= BoundingBoxWidth/2
%           PHx_RobotFrame - PHcenterX - BoundingBoxWidth/2 <= 0
HindLegKine1 = PHx_RobotFrame - PHcenterX - BoundingBox_Width/2;
%           Kinematics Constraint 2:
%           PHy_RobotFrame - PHcenterY <= BoundingBoxHeight/2
%           PHy_RobotFrame - PHcenterY - BoundingBoxHeight/2 <= 0
HindLegKine2 = PHy_RobotFrame - PHcenterY - BoundingBox_Height/2;
%           Kinematics Constraint 3:
%           PHx_RobotFrame - PHcenterX >= -BoundingBoxWidth/2
%           -PHx_RobotFrame + PHcenterX - BoundingBoxWidth/2 <= 0
HindLegKine3 = -PHx_RobotFrame + PHcenterX - BoundingBox_Width/2;
%           Kinematics Constraint 4:
%           PHy_RobotFrame - PHcenterY >= -BoundingBoxHeight/2
%           -PHy_RobotFrame + PHcenterY - BoundingBoxHeight/2 <= 0
HindLegKine4 = -PHy_RobotFrame + PHcenterY - BoundingBox_Height/2;
%-------------------------------------------------------------------------
%   Collect Kinematics Constraint
KineCon = [FrontLegKine1;FrontLegKine2;FrontLegKine3;FrontLegKine4;...
           HindLegKine1;HindLegKine2;HindLegKine3;HindLegKine4];
%-------------------------------------------------------------------------

%       Friction Cone
%-------------------------------------------------------------------------
%           Front Leg
%               sqrt(FFx^2 + FFy^2) <= miu*f_n, f_n =
%               [FFx,FFy]'*[TerrainNormx,TerrainNormy] ---> in 3D
%               sqrt(FFx^2 + FFy^2) - miu*f_n <= 0
FrictionFrontLeg = FFx_current - miu.*(TerrainNorm(1).*FFx_current + TerrainNorm(2).*FFy_current);
%-------------------------------------------------------------------------
%           Hind Leg
%               sqrt(FHx^2 + FHy^2) <= miu*f_n, f_n =
%               [FHx,FHy]'*[TerrainNormx,TerrainNormy] ---> in 3D
%               sqrt(FHx^2 + FHy^2) - miu*f_n <= 0
FrictionHindLeg = FHx_current - miu.*(TerrainNorm(1).*FHx_current + TerrainNorm(2).*FHy_current);
%-------------------------------------------------------------------------
%   Collect Friction Cone Constraint
FrictionCone = [FrictionFrontLeg;FrictionHindLeg];

c = [KineCon;FrictionCone];
ceq = [x_pos_dyn;x_vel_dyn;...
       y_pos_dyn;y_vel_dyn;...
       theta_pos_dyn;theta_vel_dyn;...
       PFx_pos_dyn;PFy_pos_dyn;
       PHx_pos_dyn;PHy_pos_dyn];%[ceq_rotation_dynamics];
%ceq = [];

end