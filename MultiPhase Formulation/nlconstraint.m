function [c,ceq] = nlconstraint(vars,...
                                h,...
                                I,...
                                xIdx_init,xIdx_end,...
                                yIdx_init,yIdx_end,...
                                thetaIdx_init,thetaIdx_end,...
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

%-------------------------------------------------------------------------
%       Rotation dynamics --> I*thetadotodt = (PF - r) X FF + (PH - r) X FH
%-------------------------------------------------------------------------


x = vars(xIdx_init:xIdx_end-1);
y = vars(yIdx_init:yIdx_end-1);
theta = vars(thetaIdx_init:thetaIdx_end-1);
thetadot_future = vars(thetadotIdx_init+1:thetadotIdx_end); %tehtadot k+1
thetadot_current = vars(thetadotIdx_init:thetadotIdx_end-1);%thetadot k
PFx = vars(PFxIdx_init:PFxIdx_end-1);
PFy = vars(PFyIdx_init:PFyIdx_end-1);
PHx = vars(PHxIdx_init:PHxIdx_end-1);
PHy = vars(PHyIdx_init:PHyIdx_end-1);
FFx = vars(FFxIdx_init:FFxIdx_end-1);
FFy = vars(FFyIdx_init:FFyIdx_end-1);
FHx = vars(FHxIdx_init:FHxIdx_end-1);
FHy = vars(FHyIdx_init:FHyIdx_end-1);

%-------------------------------------------------------------------------
%       Build constraint
cross_product = (PFx - x).*FFy - (PFy - y).*FFx + (PHx - x).*FHy - (PHy - y).*FHx;
ceq_rotation_dynamics = I.*thetadot_future - I.*thetadot_current - h.*cross_product;
%-------------------------------------------------------------------------
%       Kinematics Constraint
%           Move to tensor-based formulation to speed up computation and
%           problem formulation --> Ask Chris and Theo
%-------------------------------------------------------------------------
%       R = [cos(theta), -sin(theta);
%            sin(theta), cos(theta)]
%************************************************************
%       R^T (rotation from world frame to local frame) = 
%         = [cos(theta), sin(theta)
%            -sin(theta),cos(theta)]
%************************************************************
%-------------------------------------------------------------------------
%       Front Leg:
PFx_RobotFrame = cos(theta).*(PFx - x) + sin(theta).*(PFy - y);
PFy_RobotFrame = -sin(theta).*(PFx - x) + cos(theta).*(PFy - y);
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
PHx_RobotFrame = cos(theta).*(PHx - x) + sin(theta).*(PHy - y);
PHy_RobotFrame = -sin(theta).*(PHx - x) + cos(theta).*(PHy - y);
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
FrictionFrontLeg = FFx - miu.*(TerrainNorm(1).*FFx + TerrainNorm(2).*FFy);
%-------------------------------------------------------------------------
%           Hind Leg
%               sqrt(FHx^2 + FHy^2) <= miu*f_n, f_n =
%               [FHx,FHy]'*[TerrainNormx,TerrainNormy] ---> in 3D
%               sqrt(FHx^2 + FHy^2) - miu*f_n <= 0
FrictionHindLeg = FHx - miu.*(TerrainNorm(1).*FHx + TerrainNorm(2).*FHy);
%-------------------------------------------------------------------------
%   Collect Friction Cone Constraint
FrictionCone = [FrictionFrontLeg;FrictionHindLeg];

c = [KineCon;FrictionCone];
ceq = [ceq_rotation_dynamics];
%ceq = [];

end