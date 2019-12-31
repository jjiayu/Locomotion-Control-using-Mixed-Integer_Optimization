function [g, lbg, ubg] = Constraint_Task_and_Periodicity(x,         y,         z,...           %Linear Position
                                                         xdot,      ydot,      zdot,...        %Linear Velocity
                                                         phi,       theta,     psi,...         %Orientation
                                                         phidot,    thetadot,  psidot,...      %Euler Angle Rate
                                                         Plfx,      Plfy,      Plfz,...        %Left Front Feet Location (lf)
                                                         Plhx,      Plhy,      Plhz,...        %Left Hind Feet Location (lh)
                                                         Prfx,      Prfy,      Prfz,...        %Right Front Feet Location (rf)
                                                         Prhx,      Prhy,      Prhz,...        %Right Hind Feet Location (rh)
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
    % Y-axis Positions
    %---------------
    EqTemp = y(1) - y(end);
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
    % X-axis Velocity
    %---------------
    EqTemp = xdot(1) - xdot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Y-axis Velocity
    %---------------
    EqTemp = ydot(1) - ydot(end);
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
    % Roll - Orientations
    %---------------
    EqTemp = phi(1) - phi(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Pitch - Orientations
    %---------------
    EqTemp = theta(1) - theta(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Yaw - Orientations
    %---------------
    EqTemp = psi(1) - psi(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Roll dot - Euler Angle Rate
    %---------------
    EqTemp = phidot(1) - phidot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Pitch dot - Euler Angle Rate
    %---------------
    EqTemp = thetadot(1) - thetadot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Yaw dot - Euler Angle Rate
    %---------------
    EqTemp = psidot(1) - psidot(end);
    g = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %---------------
    % Leg Positions
    %---------------
    %   Rotation matrix from Robot Frame to World Frame at time step zero, k = 1 in matlab
    RTheta_0 = EulerAngle_to_RotationMatrix(phi, theta, psi, 1);
    %   Rotation matrix from Robot Frame to World Frame at terminal time step, k = length(whatever state variable) in matlab
    RTheta_end = EulerAngle_to_RotationMatrix(phi, theta, psi, length(x));
    
    %   Left Front (lf)
    EqTemp = RTheta_0'  *([Plfx(1);  Plfy(1);  Plfz(1)]    - [x(1);  y(1);  z(1)]) - ...
             RTheta_end'*([Plfx(end);Plfy(end);Plfz(end)]  - [x(end);y(end);z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2), EqTemp(3)};
    lbg = [lbg;      0;        0;          0];
    ubg = [ubg;      0;        0;          0];
    
    %   Left Hind (lh)
    EqTemp = RTheta_0'  *([Plhx(1);  Plhy(1);  Plhz(1)]    - [x(1);  y(1);  z(1)]) - ...
             RTheta_end'*([Plhx(end);Plhy(end);Plhz(end)]  - [x(end);y(end);z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2), EqTemp(3)};
    lbg = [lbg;      0;        0;          0];
    ubg = [ubg;      0;        0;          0];
    
    %   Right Front (rf)
    EqTemp = RTheta_0'  *([Prfx(1);  Prfy(1);  Prfz(1)]    - [x(1);  y(1);  z(1)]) - ...
             RTheta_end'*([Prfx(end);Prfy(end);Prfz(end)]  - [x(end);y(end);z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2), EqTemp(3)};
    lbg = [lbg;      0;        0;          0];
    ubg = [ubg;      0;        0;          0];
    
    %   Right Hind (rh)
    EqTemp = RTheta_0'  *([Prhx(1);  Prhy(1);  Prhz(1)]    - [x(1);  y(1);  z(1)]) - ...
             RTheta_end'*([Prhx(end);Prhy(end);Prhz(end)]  - [x(end);y(end);z(end)]);
    g = {g{:}, EqTemp(1), EqTemp(2), EqTemp(3)};
    lbg = [lbg;      0;        0;          0];
    ubg = [ubg;      0;        0;          0];
         
    %---------------
    % Periodicity in Contact Configurations
    %--------------- 
    %   Build Equation first
    Eqtemp_Clf = 0; %Left Front (lf)
    Eqtemp_Clh = 0; %Left Hind (lh)
    Eqtemp_Crf = 0; %Right Front (rf)
    Eqtemp_Crh = 0; %Right Hind (rh)
    for i = 1:length(Clf) - 1
        %we can use abs function/square function to remove the minus sign
        %due to 0-1 and 1-0
        
        %ABS Version
        Eqtemp_Clf = Eqtemp_Clf + abs(Clf(i+1)-Clf(i)); %Left Front (lf)
        Eqtemp_Clh = Eqtemp_Clh + abs(Clh(i+1)-Clh(i)); %Left Hind (lh)
        Eqtemp_Crf = Eqtemp_Crf + abs(Crf(i+1)-Crf(i)); %Right Front(rf)
        Eqtemp_Crh = Eqtemp_Crh + abs(Crh(i+1)-Crh(i)); %Right Hind(rh)
%         %Square Version (May have problems due to conic constraint)
%         Eqtemp_lf = Eqtemp_lf + (Clf(i+1)-Clf(i))^2; %Left Front (lf)
%         Eqtemp_lh = Eqtemp_lh + (Clh(i+1)-Clh(i))^2; %Left Hind (lh)
%         Eqtemp_rf = Eqtemp_rf + (Crf(i+1)-Crf(i))^2; %Right Front(rf)
%         Eqtemp_rh = Eqtemp_rh + (Crh(i+1)-Crh(i))^2; %Right Hind(rh)
    end
    
    g = {g{:},Eqtemp_Clf,Eqtemp_Clh,Eqtemp_Crf,Eqtemp_Crh};
    lbg = [lbg;   1;        1;        1;        1];
    ubg = [ubg;   2;        2;        2;        2];
   
end

