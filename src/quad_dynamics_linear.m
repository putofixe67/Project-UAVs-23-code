function xdot = quad_dynamics_linear(x, m, g, u)

    % --- State unpack ---
    p = x(1:3);
    v = x(4:6);
    
    % --- Thrust ---
    T = u(3)+m*g;

    % --- Angles unpack ---
    phi = -u(2)/g;
    theta = u(1)/g;

    R = [1,  0, theta;
         0,  1, -phi;
         -theta, 0, 1];

    % --- Forces ---
    fT = R*[0;0;T];
    fg =[0;0;-m*g];
    
    % Input acceleration vector
    a = (fT + fg)/m;

    % Velocity Inertial
    vdot = a;

    % Position inertial
    pdot = v;


    % --- Full state derivative ---
    xdot = [pdot;
            vdot;
            ];
end