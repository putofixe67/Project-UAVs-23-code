clear; close all; clc;

% ==========================================
% PRETTY AND CUTE AND AESTHETIC CONFIGURATION
% ==========================================
set(0, 'DefaultAxesFontName',       'Times New Roman', ...
       'DefaultAxesFontSize',        11, ...
       'DefaultTextFontName',        'Times New Roman', ...
       'DefaultLineLineWidth',       1.3, ...
       'DefaultAxesTickLabelInterp', 'latex', ...
       'DefaultTextInterpreter',     'latex');

COL = [
    0.173 0.412 0.694   % blue
    0.153 0.561 0.329   % green
    0.816 0.180 0.180   % red
    0.494 0.278 0.706   % purple
];

% ==============================
%  CRAZYFLIE DRONE STATE SPACE
% ==============================
m = 29e-3;
g = 9.81;
d = 1e-4;

% ================================
%      --- LQR Execution ---
% ================================

% Simulation setup
N = 1000;
dt = 0.01;
t = linspace(0, dt*N, N)';   % 0 a 10 segundos, vector coluna

% ===================
% --- LQR Linear ---
% ===================

A_lin =  [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_lin = [zeros(3,3); 
         eye(3)];
C_lin = eye(6);
sys = ss(A_lin, B_lin, C_lin, 0);

Q_lin = eye(6);
Q_lin(3, 3) = 10;
Q_lin(1:2, 1:2) = eye(2) * 100;

Q_lin(4, 4) = 10;
Q_lin(5, 5) = 10;
Q_lin(6, 6) = 10;

R_lin = eye(3);
K_lin = lqr(sys, Q_lin, R_lin);

% ======================
% --- LQR Nonlinear ---
% ======================


A_nl =  [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_nl = [zeros(3,3); 
         eye(3)];
C_nl = eye(6);
sys = ss(A_nl, B_nl, C_nl, 0);

Q_nl = eye(6);
Q_nl(3, 3) = 100;
Q_nl(1:2, 1:2) = eye(2) * 100;

Q_nl(4, 4) = 10;
Q_nl(5, 5) = 10;
Q_nl(6, 6) = 10;

R_nl = eye(3);
K_nl = lqr(sys, Q_nl, R_nl);

% ===================================
% --- LQR Error Space Execution ---
% ===================================

A_error = [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_error = [zeros(3,3); eye(3)];
C_error = eye(6);

sys = ss(A_error, B_error, C_error, 0);

Q_LQR_ES = eye(6);
Q_LQR_ES(3, 3) = 100;
Q_LQR_ES(1:2, 1:2) = eye(2) * 100;

Q_LQR_ES(4, 4) = 10;
Q_LQR_ES(5, 5) = 10;
Q_LQR_ES(6, 6) = 10;

R_LQR_ES = eye(3);

K_LQR_ES = lqr(sys, Q_LQR_ES, R_LQR_ES);

% ================================
%   --- Lyapunov Execution ---
% ================================

% Gains — Kv > I ensures V_dot < 0 (Lyapunov stability)
Kp = 10 * eye(3);

Kv = 6 * eye(3);
Kv(1, 1) = 6;

% ====================================================
%  FLAGS: ligar/desligar feedforward de velocidade e aceleração
% ====================================================
USE_V_DESIRED = true;   % true  -> usa velocidade desejada na referência
                        % false -> velocidade desejada = 0
USE_A_DESIRED = true;   % true  -> adiciona aceleração desejada ao comando
                        % false -> comando = -K*(x - xd)

% ====================================================
%  TRAJECTÓRIA ESPIRAL (analítica)
% ====================================================
R     = 1.0;          % raio da espiral [m]
omega = 2*pi*0.1;     % frequência angular -> 2 voltas completas em 10 s
v_z   = 0.1;          % velocidade de subida [m/s]

% Posição desejada
px_ref = R * cos(omega * t);
py_ref = R * sin(omega * t);
pz_ref = v_z * t;

% Velocidade desejada (derivada analítica)
vx_ref = -R * omega * sin(omega * t);
vy_ref =  R * omega * cos(omega * t);
vz_ref =  v_z * ones(size(t));

% Aceleração desejada (derivada segunda)
ax_ref = -R * omega^2 * cos(omega * t);
ay_ref = -R * omega^2 * sin(omega * t);
az_ref =  0 * ones(size(t));

% Matrizes de referência (já são colunas porque t é coluna)
p_desired = [px_ref, py_ref, pz_ref];
v_desired = [vx_ref, vy_ref, vz_ref];
a_desired = [ax_ref, ay_ref, az_ref];

