function [g,lbg, ubg] = Constraint_SystemDynamics(m,         I,        G,...           %Inertia Info
                                                  x,         z,        theta,...     %Decision Variables
                                                  xdot,      zdot,     thetadot,...
                                                  Plfx,      Plfz,...
                                                  Plhx,      Plhz,...
                                                  Prfx,      Prfz,...
                                                  Prhx,      Prhz,...
                                                  Plfxdot,   Plfzdot,...
                                                  Plhxdot,   Plhzdot,...
                                                  Prfxdot,   Prfzdot,...
                                                  Prhxdot,   Prhzdot,...
                                                  Flfx,      Flfz,...
                                                  Flhx,      Flhz,...
                                                  Frfx,      Frfz,...
                                                  Frhx,      Frhz,...
                                                  h,...                             %No Switching time vector, instead put Time Steps
                                                  k)                                %Knot Number


    %Initialize Constraint Containers
    g = {};
    lbg = [];
    ubg = [];
    
    %Start Building Constraints
    %------------------
    %   Torso Dynamics
    %------------------
    %    x-axis velocity (1st-order)
    %------------------
    EqTemp = x(k+1) - x(k) - h*xdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    x-axis Dynamics (2nd-order)
    %------------------
    EqTemp = xdot(k+1) - xdot(k) -h*(Flfx(k)/m + Flhx(k)/m + Frfx(k)/m + Frhx(k)/m);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    z-axis velocity (1st-order)
    %------------------
    EqTemp = z(k+1) - z(k) - h*zdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    z-axis Dynamics (2nd-order)
    %------------------
    EqTemp = zdot(k+1) - zdot(k) -h*(Flfz(k)/m + Flhz(k)/m + Frfz(k)/m + Frhz(k)/m-G);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    theta-axis(Rotation Axis) velocity (1st-order)
    %------------------
    EqTemp = theta(k+1) - theta(k) - h*thetadot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    theta-axis(Rotation Axis) Dynamics (2st-order)
    %       Cross Product: a = [ax,ay], b = [bx,by]
    %                      a x b = ax*by - ay*bx
    %------------------
    EqTemp = I*thetadot(k+1) - I*thetadot(k) - h*( (Plfx(k) - x(k))*Flfz(k) - (Plfz(k) - z(k))*Flfx(k) + ...
                                                   (Plhx(k) - x(k))*Flhz(k) - (Plhz(k) - z(k))*Flhx(k) + ...
                                                   (Prfx(k) - x(k))*Frfz(k) - (Prfz(k) - z(k))*Frfx(k) + ...
                                                   (Prhx(k) - x(k))*Frhz(k) - (Prhz(k) - z(k))*Frhx(k)   ...
                                                 );
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %------------------
    %   End-Effector Dynamics
    %------------------
    %    Left Front (lf) x-axis
    %------------------
    EqTemp = Plfx(k+1) - Plfx(k) - h*Plfxdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Left Front (lf) z-axis
    %------------------
    EqTemp = Plfz(k+1) - Plfz(k) - h*Plfzdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Left Hind (lh) x-axis
    %------------------
    EqTemp = Plhx(k+1) - Plhx(k) - h*Plhxdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Left Hind (lh) z-axis
    %------------------
    EqTemp = Plhz(k+1) - Plhz(k) - h*Plhzdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Right Front (rf) x-axis
    %------------------
    EqTemp = Prfx(k+1) - Prfx(k) - h*Prfxdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Right Front (rf) z-axis
    %------------------
    EqTemp = Prfz(k+1) - Prfz(k) - h*Prfzdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Right Hind (rh) x-axis
    %------------------
    EqTemp = Prhx(k+1) - Prhx(k) - h*Prhxdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Right Hind (rh) z-axis
    %------------------
    EqTemp = Prhz(k+1) - Prhz(k) - h*Prhzdot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
                                                              
end

