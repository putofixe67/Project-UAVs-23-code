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

COL = {[0.173 0.412 0.694], [0.153 0.561 0.329], [0.816 0.180 0.180]};

% ==============================
%  CRAZYFLIE DRONE STATE SPACE
% ==============================
m = 29e-3;
g = 9.81;

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

% --- Simulation setup ---
N = 1000;
dt = 0.01;
t = linspace(0, dt*N, N)';   % 0 a 10 segundos, vector coluna

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

% --- Inicialização ---
x_nl = zeros(N,6);
x_lin = zeros(N,6);
u_nl = zeros(3,N);
u_lin = zeros(3,N);

% --- Simulação Não Linear ---
for k = 1:N-1
    % Referência de estado desejado
    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; 0; 0; 0];
    end
    
    xk = x_nl(k,:)';
    
    % Lei de controlo
    uk = -K*(xk - xd);
    if USE_A_DESIRED
        uk = uk + a_desired(k,:)';
    end
    
    u_nl(:, k) = uk;
    xdot = quad_dynamics_nonlinear(xk, m, g, uk);
    x_nl(k+1,:) = xk' + dt*xdot';
end
u_nl(:, N) = u_nl(:, N-1);

% --- Simulação Linear ---
for k = 1:N-1
    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; 0; 0; 0];
    end
    
    xk = x_lin(k,:)';
    
    uk = -K*(xk - xd);
    if USE_A_DESIRED
        uk = uk + a_desired(k,:)';
    end
    
    u_lin(:, k) = uk;
    xdot = quad_dynamics_linear(xk, m, g, uk);
    x_lin(k+1,:) = xk' + dt*xdot';
end
u_lin(:, N) = u_lin(:, N-1);

% ==========================================
% PLOTS
% ==========================================

% --- Figura 1: 3D Trajectory ---
fig1 = figure('Name', '3D Trajectory', 'Color', 'w', 'Position', [50 100 600 500]);
ax1 = axes(fig1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
plot3(ax1, p_desired(:,1), p_desired(:,2), p_desired(:,3), 'k--', ...
      'LineWidth', 1.5, 'DisplayName', 'Reference');
plot3(ax1, x_nl(:,1), x_nl(:,2), x_nl(:,3), ...
      'Color', COL{1}, 'LineWidth', 1.8, 'DisplayName', 'Nonlinear');
plot3(ax1, x_lin(:,1), x_lin(:,2), x_lin(:,3), '-.', ...
      'Color', COL{3}, 'LineWidth', 1.8, 'DisplayName', 'Linear');
view(ax1, 30, 30);
legend(ax1, 'Location', 'best');
xlabel(ax1, '$p_x$ [m]'); ylabel(ax1, '$p_y$ [m]'); zlabel(ax1, '$p_z$ [m]');
title(ax1, '3D Trajectory Comparison');

% --- Figura 2: Position Comparison ---
fig2 = figure('Name', 'Position States', 'Color', 'w', 'Position', [670 500 500 500]);
tlo1 = tiledlayout(fig2, 3, 1, 'TileSpacing', 'compact');
p_labels = {'$p_x$ [m]', '$p_y$ [m]', '$p_z$ [m]'};
for i = 1:3
    ax = nexttile(tlo1); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    plot(ax, t, p_desired(:, i), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    plot(ax, t, x_nl(:, i), 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    plot(ax, t, x_lin(:, i), '--', 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Linear');
    ylabel(ax, p_labels{i}); applyPadding(ax, 0.15);
    if i == 1, title(ax, 'Position State Vectors'); legend(ax, 'Location', 'best'); end
    if i == 3, xlabel(ax, '$t$ [s]'); end
end

% --- Figura 3: Velocity Comparison ---
fig3 = figure('Name', 'Velocity States', 'Color', 'w', 'Position', [1190 500 500 500]);
tlo2 = tiledlayout(fig3, 3, 1, 'TileSpacing', 'compact');
v_labels = {'$v_x$ [m/s]', '$v_y$ [m/s]', '$v_z$ [m/s]'};
for i = 1:3
    ax = nexttile(tlo2); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    if USE_V_DESIRED
        plot(ax, t, v_desired(:, i), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    plot(ax, t, x_nl(:, i+3), 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    plot(ax, t, x_lin(:, i+3), '--', 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Linear');
    ylabel(ax, v_labels{i}); applyPadding(ax, 0.15);
    if i == 1, title(ax, 'Velocity State Vectors'); legend(ax, 'Location', 'best'); end
    if i == 3, xlabel(ax, '$t$ [s]'); end
end

% --- Figura 4: Actuation Inputs ---
fig4 = figure('Name', 'Actuation Inputs', 'Color', 'w', 'Position', [670 50 1020 380]);
tlo3 = tiledlayout(fig4, 1, 3, 'TileSpacing', 'compact');
u_labels = {'$u_1$ ($\sim\theta$ cmd)', '$u_2$ ($\sim\phi$ cmd)', '$u_3$ ($\Delta T$ cmd)'};
for i = 1:3
    ax = nexttile(tlo3); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    plot(ax, t, u_nl(i, :), 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    plot(ax, t, u_lin(i, :), '--', 'Color', COL{i}, 'LineWidth', 1.5, 'DisplayName', 'Linear');
    ylabel(ax, u_labels{i}); applyPadding(ax, 0.15);
    xlabel(ax, '$t$ [s]');
    if i == 2, title(ax, 'Control Actuation Vectors'); end
    if i == 1, legend(ax, 'Location', 'best'); end
end

% ==========================================
% FUNÇÕES AUXILIARES
% ==========================================

function xdot = quad_dynamics_nonlinear(x, m, g, u)
    v = x(4:6);
    ax = u(1); ay = u(2); az = u(3);
    
    theta = ax / g;
    phi   = -ay / g;
    
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
    v = x(4:6);
    T = u(3) + m*g;
    
    phi = -u(2)/g;
    theta = u(1)/g;
    
    R = [1,  0, theta;
         0,  1, -phi;
        -theta, phi, 1]; 
         
    fT = R*[0;0;T];
    fg = [0;0;-m*g];
    
    a = (fT + fg)/m;
    xdot = [v; a];
end

function applyPadding(ax, percent)
    lineObj = findobj(ax, 'Type', 'line');
    if isempty(lineObj), return; end
    y_data = [lineObj.YData];
    y_range = max(y_data) - min(y_data);
    if y_range == 0, y_range = 1e-3; end
    ylim(ax, [min(y_data) - percent * y_range, max(y_data) + percent * y_range]);
end