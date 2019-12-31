function [g,lbg, ubg] = Constraint_SystemDynamics(m,         Ib,        G,...           %Inertia Info
                                                  x,         y,         z,...           %Linear Position
                                                  xdot,      ydot,      zdot,...        %Linear Velocity
                                                  phi,       theta,     psi,...         %Orientation
                                                  phidot,    thetadot,  psidot,...      %Euler Angle Rate
                                                  Plfx,      Plfy,      Plfz,...        %Left Front Feet Location (lf)
                                                  Plhx,      Plhy,      Plhz,...        %Left Hind Feet Location (lh)
                                                  Prfx,      Prfy,      Prfz,...        %Right Front Feet Location (rf)
                                                  Prhx,      Prhy,      Prhz,...        %Right Hind Feet Location (rh)
                                                  Plfxdot,   Plfydot,   Plfzdot,...     %Left Front Feet Velocity (lf)
                                                  Plhxdot,   Plhydot,   Plhzdot,...     %Left Hind Feet Velocity (lh)
                                                  Prfxdot,   Prfydot,   Prfzdot,...     %Right Front Feet Velocity (rf)
                                                  Prhxdot,   Prhydot,   Prhzdot,...     %Right Hind Feet Velocity (rh)
                                                  Flfx,      Flfy,      Flfz,...        %Left Front Feet Force (lf)
                                                  Flhx,      Flhy,      Flhz,...        %Left Hind Feet Force (lh)
                                                  Frfx,      Frfy,      Frfz,...        %Right Front Feet Force (rf)
                                                  Frhx,      Frhy,      Frhz,...        %Right Hind Feet Force (rh)
                                                  h,...                             %No Switching time vector, instead put Time Steps
                                                  RTheta,...                        %Rotated Matrix Converted from Euler Angles
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
    %    y-axis velocity (1st-order)
    %------------------
    EqTemp = y(k+1) - y(k) - h*ydot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    y-axis Dynamics (2nd-order)
    %------------------
    EqTemp = ydot(k+1) - ydot(k) -h*(Flfy(k)/m + Flhy(k)/m + Frfy(k)/m + Frhy(k)/m);
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
    %    Roll (phi) - Angular Velocity (1st-order)
    %------------------
    EqTemp = phi(k+1) - phi(k) - h*phidot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Pitch (theta) - Angular Velocity (1st-order)
    %------------------
    EqTemp = theta(k+1) - theta(k) - h*thetadot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    %------------------
    %    Yaw (theta) - Angular Velocity (1st-order)
    %------------------
    EqTemp = psi(k+1) - psi(k) - h*psidot(k);
    g   = {g{:}, EqTemp};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    %------------------
    %    Angular Dynamics (2st-order)
    %       Cross Product: a = [ax,ay,az], b = [bx,by,bz]
    %                      a x b = [ay*bz - az*by;
    %                               az*bx - ax*bz;
    %                               ax*by - ay*bx]
    %------------------
    %Inertia Tensor in world frame
    Iw_k = RTheta*Ib*RTheta';
    
    %Euler Angle Rates 
    Thetadot_k      = [phidot(k);  thetadot(k);  psidot(k)]; %Euler Angle Rate -> The current time step k
    Thetadot_k_next = [phidot(k+1);thetadot(k+1);psidot(k+1)]; %Euler Angle Rate -> The next Time Step k+1
    
    %Lamped Robot Torso Linear Positions
    r_k = [x(k); y(k); z(k)];
    
    %Lamped Feet Location Vectors
    Plf_k = [Plfx(k); Plfy(k); Plfz(k)]; %Left Front (lf)
    Plh_k = [Plhx(k); Plhy(k); Plhz(k)]; %Left Hind (lh)
    Prf_k = [Prfx(k); Prfy(k); Prfz(k)]; %Right Front (rf)
    Prh_k = [Prhx(k); Prhy(k); Prhz(k)]; %Right Hind(rh)
    
    %Lamped Feet Force Vectors
    Flf_k = [Flfx(k); Flfy(k); Flfz(k)]; %Left Front (lf)
    Flh_k = [Flhx(k); Flhy(k); Flhz(k)]; %Left Hind (lh)
    Frf_k = [Frfx(k); Frfy(k); Frfz(k)]; %Right Front (rf)
    Frh_k = [Frhx(k); Frhy(k); Frhz(k)]; %Right Hind(rh)
    
    %Matrix for calculating angular velocity in world frame, a matrix
    %function of Theta only, at current time step k
    C_k = [cos(theta(k))*cos(psi(k)),         -sin(psi(k)),            0;...
           cos(theta(k))*sin(phi(k)),          cos(psi(k)),            0;...
                   -sin(theta(k))                 0,                   1];
    
    %Matrix for calculating angular accleration in world frame, a matrix
    %function of Theta and Thetadot, at time Step k
    Cdot_k = [-sin(theta(k))*cos(psi(k))*thetadot(k)-cos(theta(k))*sin(psi(k))*psidot(k),  -cos(psi(k))*psidot(k),   0;...
            -sin(theta(k))*sin(psi(k))*thetadot(k)+cos(theta(k))*cos(psi(k))*psidot(k),  -sin(psi(k))*psidot(k),   0;...
                                     -cos(theta(k))*thetadot(k),                              0,                   0];                  
    
    %Angular Velocity in World Frame at time step k
    omega_k = C_k*Thetadot_k;
    
    %Angular Dynamics Equation                             
    EqTemp = Iw_k*C_k*(Thetadot_k_next - Thetadot_k) + h*Iw_k*Cdot_k*Thetadot_k + h*cross(omega_k,(Iw_k*omega_k))...
             - h*cross((Plf_k - r_k), Flf_k)...
             - h*cross((Plh_k - r_k), Flh_k)...
             - h*cross((Prf_k - r_k), Frf_k)...
             - h*cross((Prh_k - r_k), Frh_k);
                
    g   = {g{:}, EqTemp(1), EqTemp(2), EqTemp(3)};
    lbg = [lbg;     0;         0;         0];
    ubg = [ubg;     0;         0;         0];
    
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
    %    Left Front (lf) y-axis
    %------------------
    EqTemp = Plfy(k+1) - Plfy(k) - h*Plfydot(k);
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
    %    Left Hind (lh) y-axis
    %------------------
    EqTemp = Plhy(k+1) - Plhy(k) - h*Plhydot(k);
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
    %    Right Front (rf) y-axis
    %------------------
    EqTemp = Prfy(k+1) - Prfy(k) - h*Prfydot(k);
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
    %    Right Hind (rh) y-axis
    %------------------
    EqTemp = Prhy(k+1) - Prhy(k) - h*Prhydot(k);
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

