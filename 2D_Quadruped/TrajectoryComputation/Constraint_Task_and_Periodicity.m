function [g, lbg, ubg] = Constraint_Task_and_Periodicity(x,         z,        theta,...     %Decision Variables
                                                         xdot,      zdot,     thetadot,...
                                                         Plfx,      Plfz,...
                                                         Plhx,      Plhz,...
                                                         Prfx,      Prfz,...
                                                         Prhx,      Prhz,...
                                                         Clf,...
                                                         Clh,...
                                                         Crf,...
                                                         Crh,...
                                                         Ts,        Tend,...
                                                         speed,     SpeedDirection,...
                                                         terrain_slope_rad)
%CONSTRAINT_TASK_AND_PERIODICITY Constraints for Task and Periodicity
%Specification
    
    g = {};
    lbg = [];
    ubg = [];
    
    %---------------
    % Terminal Time Constraint
    %---------------
    EqTemp = Ts(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; Tend];
    ubg = [ubg; Tend];
    
    %---------------
    % Initial x position
    %---------------
    EqTemp = x(1);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Terminal x position
    %---------------
    if SpeedDirection == 1 %Target Speed Direction is Horizontal
        EqTemp = x(end) - speed*Tend;
    elseif SpeedDirection == 2 %Target Speed Direction is Tangential to the Slope
        EqTemp = x(end) - speed*Tend*cos(terrain_slope_rad);
    end
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Z-axis Positions
    %---------------
    if SpeedDirection == 1 %Target Speed Direction is Horizontal
        EqTemp = z(end) - z(1) - speed*Tend*tan(terrain_slope_rad);
    elseif SpeedDirection == 2 %Target Speed Direction is Tangential to the Slope
        EqTemp = z(end) - z(1) - speed*Tend*sin(terrain_slope_rad);
    end
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Theta Positions
    %---------------
    EqTemp = theta(1) - theta(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % X-axis Velocity
    %---------------
    EqTemp = xdot(1) - xdot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Z-axis Velocity
    %---------------
    EqTemp = zdot(1) - zdot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Angular Velocity
    %---------------
    EqTemp = thetadot(1) - thetadot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Leg Positions
    %---------------
    %   Left Front (lf)
    EqTemp = [cos(theta(1)),    sin(theta(1));      -sin(theta(1)),     cos(theta(1))]  *([Plfx(1);     Plfz(1)]    - [x(1);    z(1)]) - ...
             [cos(theta(end)),  sin(theta(end));    -sin(theta(end)),   cos(theta(end))]*([Plfx(end);   Plfz(end)]  - [x(end);  z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2)};
    lbg = [lbg;   0;          0];
    ubg = [ubg;   0           0];
    
    %   Left Hind (lh)
    EqTemp = [cos(theta(1)),    sin(theta(1));      -sin(theta(1)),     cos(theta(1))]  *([Plhx(1);     Plhz(1)]    - [x(1);    z(1)]) - ...
             [cos(theta(end)),  sin(theta(end));    -sin(theta(end)),   cos(theta(end))]*([Plhx(end);   Plhz(end)]  - [x(end);  z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2)};
    lbg = [lbg;   0;          0];
    ubg = [ubg;   0           0];
    
    %   Right Front (rf)
    EqTemp = [cos(theta(1)),    sin(theta(1));      -sin(theta(1)),     cos(theta(1))]  *([Prfx(1);     Prfz(1)]    - [x(1);    z(1)]) - ...
             [cos(theta(end)),  sin(theta(end));    -sin(theta(end)),   cos(theta(end))]*([Prfx(end);   Prfz(end)]  - [x(end);  z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2)};
    lbg = [lbg;   0;          0];
    ubg = [ubg;   0           0];
    
    %   Right Hind (rh)
    EqTemp = [cos(theta(1)),    sin(theta(1));      -sin(theta(1)),     cos(theta(1))]  *([Prhx(1);     Prhz(1)]    - [x(1);    z(1)]) - ...
             [cos(theta(end)),  sin(theta(end));    -sin(theta(end)),   cos(theta(end))]*([Prhx(end);   Prhz(end)]  - [x(end);  z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2)};
    lbg = [lbg;   0;          0];
    ubg = [ubg;   0           0];
         
    %---------------
    % Periodicity in Contact Configurations
    %--------------- 
    %   Build Equation first
    Eqtemp_lf = 0; %Left Front (lf)
    Eqtemp_lh = 0; %Left Hind (lh)
    Eqtemp_rf = 0; %Right Front (rf)
    Eqtemp_rh = 0; %Right Hind (rh)
    for i = 1:length(Clf) - 1
        %we can use abs function/square function to remove the minus sign
        %due to 0-1 and 1-0
        
        %ABS Version
        Eqtemp_lf = Eqtemp_lf + abs(Clf(i+1)-Clf(i)); %Left Front (lf)
        Eqtemp_lh = Eqtemp_lh + abs(Clh(i+1)-Clh(i)); %Left Hind (lh)
        Eqtemp_rf = Eqtemp_rf + abs(Crf(i+1)-Crf(i)); %Right Front(rf)
        Eqtemp_rh = Eqtemp_rh + abs(Crh(i+1)-Crh(i)); %Right Hind(rh)
%         %Square Version (May have problems due to conic constraint)
%         Eqtemp_lf = Eqtemp_lf + (Clf(i+1)-Clf(i))^2; %Left Front (lf)
%         Eqtemp_lh = Eqtemp_lh + (Clh(i+1)-Clh(i))^2; %Left Hind (lh)
%         Eqtemp_rf = Eqtemp_rf + (Crf(i+1)-Crf(i))^2; %Right Front(rf)
%         Eqtemp_rh = Eqtemp_rh + (Crh(i+1)-Crh(i))^2; %Right Hind(rh)
    end
    
    g = {g{:},Eqtemp_lf,Eqtemp_lh,Eqtemp_rf,Eqtemp_rh};
    lbg = [lbg;   1;        1;        1;        1];
    ubg = [ubg;   2;        2;        2;        2];
   
end

