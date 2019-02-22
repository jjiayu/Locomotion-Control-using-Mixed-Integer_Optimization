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
                                BodyHeight,...
                                BodyLength,...
                                minLegX,...
                                maxLegX,...
                                minLegY,...
                                maxLegY)
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
%       R = [cos(theta), -sin(theta);
%            sin(theta), cos(theta)]
%       R^T (rotation from world frame to local frame) = 
%         = [cos(theta), sin(theta)
%            -sin(theta),cos(theta)]

PFx_local = cos(theta).*PFx + sin(theta).*PFy;

PFy_local = -sin(theta).*PFx + cos(theta).*PFy;

PHx_local = cos(theta).*PHx + sin(theta).*PHy;

PHy_local = -sin(theta).*PHx + cos(theta).*PHy;

%   Hind Foot
%-------------------------------------------------------------------------
%       y-axis
%           First Constraint: y - PHy_local >= BodyHeight/2 + minLegY -->
%                             y - PHy_local - BodyHeight/2 - minLegY >= 0 -->
%                             -y + PHy_local + BodyHeight/2 + minLegY <= 0 
PHy_Kine_Con1 = -y + PHy_local + BodyHeight/2 + minLegY;
%           Second Constraint: y - PHy_local <= BodyHeight/2 + minLegY + maxLegY
%                              y - PHy_local - BodyHeight/2 - minLegY - maxLegY <=0
PHy_Kine_Con2 = y - PHy_local - BodyHeight/2 - minLegY - maxLegY;
%       x-axis
%           First Constraint: x - PHx_local >= minLegX -->
%                             x - PHx_local - minLegX >= 0 -->
%                             -x + PHx_local + minLegX <= 0
PHx_Kine_Con1 = -x + PHx_local + minLegX;
%           Second Constraint: x - PHx_local <= minLegX + maxLegX
%                              x - PHx_local - minLegX - maxLegX <= 0
PHx_Kine_Con2 = x - PHx_local - minLegX - maxLegX;
%-------------------------------------------------------------------------
%   Front Foot
%       y-axis
%           First Constraint:  y - PFy_local >= BodyHeight/2 + minLegY
%                              y - PFy_local - BodyHeight/2 - minLegY >=0
%                              - y + PFy_local + BodyHeight/2 +minLegY <=0
PFy_Kine_Con1 = -y + PFy_local + BodyHeight/2 + minLegY;
%           Second Constraint: y - PFy_local <= BodyHeight/2 + minLegY + maxLegY
%                              y - PFy_local - BodyHeight/2 - minLegY - maxLegY <= 0
PFy_Kine_Con2 = y - PFy_local - BodyHeight/2 - minLegY - maxLegY;
%       x-axis
%           First Constraint: PFx_local - x >= minLegX
%                             PFx_local - x - minLegX >= 0
%                             -PFx_local + x + minLegX <= 0
PFx_Kine_Con1 = -PFx_local + x + minLegX;
%           Second Constraint: PFx - x <= minLegX + maxLegX
%                              PFx - x - minLegX - maxLegX <= 0
PFx_Kine_Con2 = PFx_local - x - minLegX - maxLegX;



c = [PHy_Kine_Con1;PHy_Kine_Con2;PHx_Kine_Con1;PHx_Kine_Con2;...
     PFy_Kine_Con1;PFy_Kine_Con2;PFx_Kine_Con1;PFx_Kine_Con2];
ceq = [ceq_rotation_dynamics];
%ceq = [];

end