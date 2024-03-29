function [c,ceq] = nlconstraint_current_future_vectors(vars,...
                                                       h,...
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
                                                       FFxIdx_init,FFxIdx_end,...
                                                       FFyIdx_init,FFyIdx_end,...
                                                       FHxIdx_init,FHxIdx_end,...
                                                       FHyIdx_init,FHyIdx_end,...
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
%       Rotation dynamics --> I*thetadotodt = (PF - r) X FF + (PH - r) X FH
%=========================================================================
%       

%-------------------------------------------------------------------------
%       Build constraint
cross_product = (PFx_current - x_current).*FFy_current - (PFy_current - y_current).*FFx_current + (PHx_current - x_current).*FHy_current - (PHy_current - y_current).*FHx_current;
ceq_rotation_dynamics = I.*thetadot_future - I.*thetadot_current - h.*cross_product;
%-------------------------------------------------------------------------

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
PFx_RobotFrame = cos(theta_current).*(PFx_current - x_current) + sin(theta_current).*(PFy_current - y_current);
PFy_RobotFrame = -sin(theta_current).*(PFx_current - x_current) + cos(theta_current).*(PFy_current - y_current);
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
PHx_RobotFrame = cos(theta_current).*(PHx_current - x_current) + sin(theta_current).*(PHy_current - y_current);
PHy_RobotFrame = -sin(theta_current).*(PHx_current - x_current) + cos(theta_current).*(PHy_current - y_current);
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
ceq = [ceq_rotation_dynamics];
%ceq = [];

end