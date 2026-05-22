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

A_lin = [zeros(3,3), eye(3); 
         zeros(3,3), zeros(3,3)];
B_lin = [zeros(3,3); 
         eye(3)];
C_lin = eye(6);
sys = ss(A_lin, B_lin, C_lin, 0);

% ================================
%      --- LQR Execution ---
% ================================
Q = eye(6);
Q(1:3, 1:3) = eye(3) * 10;
R = eye(3);
K = lqr(sys, Q, R);

% Simulation setup
N = 1000;
dt = 0.01;
t = linspace(0, dt*N, N)';   % 0 a 10 segundos, vector coluna

% ================================
%   --- Lyapunov Execution ---
% ================================

% Gains — Kv > I ensures V_dot < 0 (Lyapunov stability)
Kp = 7.0 * eye(3);
Kv = 7.0 * eye(3);

% ====================================================
%  FLAGS: ligar/desligar feedforward de velocidade e aceleração
% ====================================================
USE_V_DESIRED = true;   % true  -> usa velocidade desejada na referência
                        % false -> velocidade desejada = 0
USE_A_DESIRED = true;   % true  -> adiciona aceleração desejada ao comando
                        % false -> comando = -K*(x - xd)

% ====================================================
%  TRAJECTÓRIA ESPIRAL (analítica, sem syms)
% ====================================================
R     = 2.0;          % raio da espiral [m]
omega = 2*pi*0.2;     % frequência angular -> 2 voltas completas em 10 s
v_z   = 0.5;          % velocidade de subida [m/s]

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

