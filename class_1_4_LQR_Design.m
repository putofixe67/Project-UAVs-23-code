clear; close all; clc;

init();

%% ===============================================
%  ---              LQR Design            ---
% ================================================
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

% ==============================
%  crazyflyer drone state space
% ==============================
m = 29e-3;
g = 9.81;
d = 1e-4;

% --- Variables declaration ---
A_lin = [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_lin = [zeros(3,3); eye(3)];

% --- Consider all variables obsv ---
C_lin = eye(6);

sys = ss(A_lin, B_lin, C_lin, 0);

% ================================
%      --- LQR Execution ---
% ================================

Q = eye(6);
R = eye(3);

K = lqr(sys, Q, R);

% --- simulation setup ---
N = 1000;
dt = 0.01;

t = linspace(0, dt*N, N);

x_nl = zeros(N,6);
u = zeros(3,N);

cte = ones(1, N);

traj =[cos(t); sin(t); t]';

% --- Nonlinear ---
for k = 1:N-1
    xd = [traj(k,1); traj(k,2); traj(k,3); 0; 0; 0];
    xk = x_nl(k,:)';
    uk = -K*(xk - xd);

    xdot = quad_dynamics_nonlinear(xk, m, g, uk);
    x_nl(k+1,:) = xk' + dt*xdot';
end

x_lin = zeros(N,6);
u = zeros(3,N);

% --- Linear ---
for k = 1:N-1

    xd = [traj(k,1); traj(k,2); traj(k,3); 0; 0; 0]; 

    xk = x_lin(k,:)';
    uk = -K*(xk - xd);

    xdot = quad_dynamics_linear(xk, m, g, uk);
    x_lin(k+1,:) = xk' + dt*xdot';
end

figure;
plot3(traj(:,1), traj(:,2), traj(:,3), 'o--', 'LineWidth', 2); hold on;
plot3(x_nl(:,1), x_nl(:,2), x_nl(:,3), 'o', 'LineWidth', 1.8);
plot3(x_lin(:,1), x_lin(:,2), x_lin(:,3), 'o--', 'LineWidth', 1.8);

grid on;
legend('Reference','Nonlinear','Linear', 'Location','best');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Trajectory comparison');

figure;
plot(t, x_nl(:,4), 'b', t, x_lin(:,4), 'r--'); hold on;
plot(t, x_nl(:,5), 'b', t, x_lin(:,5), 'r--');
plot(t, x_nl(:,6), 'b', t, x_lin(:,6), 'r--');

legend('vx NL','vx LIN','vy NL','vy LIN','vz NL','vz LIN');
grid on;
title('Velocity comparison');

figure;
plot(t, x_nl(:,1), 'b', t, x_lin(:,1), 'r--'); hold on;
plot(t, x_nl(:,2), 'b', t, x_lin(:,2), 'r--');
plot(t, x_nl(:,3), 'b', t, x_lin(:,3), 'r--');

legend('px NL','px LIN','py NL','py LIN','pz NL','pz LIN');
grid on;
title('Position comparison');

% ===================================
% --- LQR Error Spaace Execution ---
% ===================================

A_error = [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_error = [zeros(3,3); eye(3)];
C_error = eye(6);

sys = ss(A_error, B_error, C_error, 0);

Q = eye(6);
R = eye(3);

K = lqr(sys, Q, R);

% States: [px py pz vx vy vz]
x_error = zeros(N,6);

% Inputs are 3D
u_error = zeros(N,3);

x = zeros(N,6);

% Desired feedforward input
ud = zeros(3,N);

for k = 1:N-1

    % Desired state
    xd = [traj(k,1); traj(k,2); traj(k,3); 0; 0; 0];

    xk = x(k,:)';

    ek = xk - xd;

    x_error(k,:) = ek';

    u_error(k,:) = (-K * ek)';

    uk = ud(:,k) + u_error(k,:)';

    xdot = quad_dynamics_nonlinear(xk, m, g, uk);

    x(k+1,:) = xk' + dt * xdot';
end

figure;
plot3(traj(:,1), traj(:,2), traj(:,3), 'o--', 'LineWidth', 2); hold on;
plot3(x_nl(:,1), x_nl(:,2), x_nl(:,3), 'o', 'LineWidth', 1.8);
plot3(x_lin(:,1), x_lin(:,2), x_lin(:,3), 'o--', 'LineWidth', 1.8);
plot3(x(:,1), x(:,2), x(:,3), 'o--', 'LineWidth', 1.8);

grid on;
legend('Reference','Nonlinear','Linear', 'Location','best');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Trajectory comparison');

figure;
plot(t, x(:,4), 'b', t, x_lin(:,4), t, x_nl(:,4), 'r--'); hold on;
plot(t, x(:,5), 'b', t, x_lin(:,5), t, x_nl(:,5), 'r--');
plot(t, x(:,6), 'b', t, x_lin(:,6), t, x_nl(:,6), 'r--');

legend('vx NL','vx LIN','vx ErrSpace', 'vy NL','vy LIN', ...
    'vy ErrSpace','vz NL','vz LIN','vz ErrSpace');

grid on;
title('Velocity comparison');

figure;
plot(t, x(:,1), 'b', t, x_lin(:,1), t, x_nl(:,1), 'r--'); hold on;
plot(t, x(:,2), 'b', t, x_lin(:,2), t, x_nl(:,1), 'r--');
plot(t, x(:,3), 'b', t, x_lin(:,3), t, x_nl(:,1), 'r--');

legend('px NL','px LIN','px ErrSpace', 'py NL','py LIN', ...
       'py ErrSpace', 'pz NL','pz LIN', 'pz ErrSpace');
grid on;
title('Position comparison');
