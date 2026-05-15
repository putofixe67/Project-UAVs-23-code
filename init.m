clear; close all; clc;

%% ================= Crazyflie Physical Parameters =================
m  = 29e-3;                     % mass (kg)
g  = 9.81;                      % gravity (m/s^2)
% Inertia matrix (approximate Crazyflie 2.0 values)
Jx = 1.395e-5;   Jy = 1.436e-5;   Jz = 2.173e-5;
J  = diag([Jx, Jy, Jz]);

%% ================= Symbolic State & Input Definition =================
% State: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]'
syms x y z vx vy vz phi theta psi wx wy wz real
state = [x; y; z; vx; vy; vz; phi; theta; psi; wx; wy; wz];

% Inputs: total thrust (F) and body torques (Mx, My, Mz)
syms F Mx My Mz real
u = [F; Mx; My; Mz];

% Parameters (keep symbolic to allow substitution later)
syms m_sym g_sym Jx_sym Jy_sym Jz_sym real
Jsym = diag([Jx_sym, Jy_sym, Jz_sym]);

%% ================= Nonlinear Equations of Motion =================
% --- Rotation matrix (ZYX Euler) ---
cp = cos(phi);   sp = sin(phi);
ct = cos(theta); st = sin(theta);
cy = cos(psi);   sy = sin(psi);
R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
     sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
     -st,    ct*sp,              ct*cp            ];

% Translational dynamics
v = [vx; vy; vz];
p_dot   = v;
v_dot   = [0; 0; -g_sym] + (1/m_sym) * R * [0; 0; F];

% Rotational kinematics (ZYX Euler rates)
W = [1,  sp*tan(theta),  cp*tan(theta);
     0,  cp,            -sp;
     0,  sp/ct,          cp/ct         ];
eul_dot = W * [wx; wy; wz];

% Rotational dynamics (Euler's equation)
omega = [wx; wy; wz];
omega_skew = [0, -wz, wy; wz, 0, -wx; -wy, wx, 0];
omega_dot = Jsym \ ( [Mx; My; Mz] - omega_skew * Jsym * omega );

% Full state derivative
f_sym = [p_dot; v_dot; eul_dot; omega_dot];

%% ================= Linearisation at Hover =================
% Equilibrium: all states zero, F = m*g, torques zero
eq_vals = [x,0; y,0; z,0; vx,0; vy,0; vz,0;
           phi,0; theta,0; psi,0; wx,0; wy,0; wz,0;
           F, m_sym*g_sym; Mx,0; My,0; Mz,0];

f_eq = subs(f_sym, eq_vals(:,1), eq_vals(:,2));   % substitute into dynamics
f_eq = simplify(f_eq);

A_sym = jacobian(f_sym, state);       % A = ∂f/∂x
B_sym = jacobian(f_sym, u);           % B = ∂f/∂u

% The equilibrium column (eq_vals(:,2)) already contains:
%   [0;0;0; 0;0;0; 0;0;0; 0;0;0; m*g; 0;0;0]
% which matches [state; u] exactly.
A_eq = subs(A_sym, [state; u], eq_vals(:,2));
B_eq = subs(B_sym, [state; u], eq_vals(:,2));

% Substitute numeric physical parameters
A_num = double(subs(A_eq, [m_sym, g_sym, Jx_sym, Jy_sym, Jz_sym], ...
                         [m,    g,    Jx,    Jy,    Jz]));
B_num = double(subs(B_eq, [m_sym, g_sym, Jx_sym, Jy_sym, Jz_sym], ...
                         [m,    g,    Jx,    Jy,    Jz]));

% Display the state-space matrices
disp('Linearised A matrix (12x12):');
disp(A_num);
disp('Linearised B matrix (12x4):');
disp(B_num);

%% ================= LQR Design Example (optional) =================
% Weighting matrices (tune as needed)
Q = diag([10, 10, 100, 1, 1, 5, 1, 1, 1, 0.1, 0.1, 0.1]);  % state penalty
R = diag([0.1, 1, 1, 1]);                                     % control penalty

% Check controllability
Co = ctrb(A_num, B_num);
if rank(Co) == size(A_num,1)
    K = lqr(A_num, B_num, Q, R);
    disp('LQR gain matrix K:');
    disp(K);
else
    disp('System not fully controllable – check model/linearisation.');
end

%% ================= Supporting Functions =================
% The functions below are corrected versions of your original ones.
% They describe a **simplified** quadcopter model (position/velocity only),
% treating attitude and thrust as external inputs.

function xdot = quad_dynamics_nonlinear(x, m, g, lambda, T)
    % State: [px; py; pz; vx; vy; vz]
    % lambda = [phi; theta; psi]  (commanded angles)
    % T = total thrust
    p = x(1:3);
    v = x(4:6);

    phi = lambda(1);
    theta = lambda(2);
    psi = lambda(3);       % semicolon added

    % Full rotation matrix (ZYX)
    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi); sy = sin(psi);
    R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
         sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
         -st,    ct*sp,             ct*cp];

    fT = R * [0; 0; T];
    fg = [0; 0; -m*g];
    u  = (fT + fg) / m;

    pdot = v;
    vdot = u;

    xdot = [pdot; vdot];
end

function xdot = quad_dynamics_linear(x, m, g, lambda, T)
    % Linearised translational dynamics around hover,
    % assuming small phi, theta and psi = 0.
    % State: [px; py; pz; vx; vy; vz]
    % lambda = [phi; theta]   (psi is ignored here, set to 0)
    p = x(1:3);
    v = x(4:6);

    phi = lambda(1);
    theta = lambda(2);
    % Linear rotation matrix (psi = 0)
    R = [1,  0, theta;
         0,  1, -phi;
         -theta, 0, 1];

    fT = R * [0; 0; T];
    fg = [0; 0; -m*g];
    u  = (fT + fg) / m;

    pdot = v;
    vdot = u;

    xdot = [pdot; vdot];
end