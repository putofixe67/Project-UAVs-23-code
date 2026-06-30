function xdot = quad_dynamics_nonlinear(x, m, g, u)

    v = x(4:6);

    ax = u(1);
    ay = u(2);
    az = u(3);

    theta = ax / g;
    phi   = -ay / g;
    psi   = 0;

    T = m*(az + g);

    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);

    R = [ct, sp*st, cp*st;
         0,   cp,   -sp;
         -st, sp*ct, cp*ct];

    fT = R*[0;0;T];
    fg = [0;0;-m*g];

    a = (fT + fg)/m;

    xdot = [v; a];
end
