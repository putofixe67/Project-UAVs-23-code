clear; close all; clc;

init();

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
      'Color', COL(1,:), 'LineWidth', 1.8, 'DisplayName', 'Nonlinear');

plot3(ax1, x_lin(:,1), x_lin(:,2), x_lin(:,3), '-.', ...
      'Color', COL(3,:), 'LineWidth', 1.8, 'DisplayName', 'Linear');

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
    
    plot(ax, t, x_nl(:, i), 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    
    plot(ax, t, x_lin(:, i), '--', 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Linear');
    
    ylabel(ax, p_labels{i});
  
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
    
    plot(ax, t, x_nl(:, i+3), 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    
    plot(ax, t, x_lin(:, i+3), '--', 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Linear');
    
    ylabel(ax, v_labels{i});
    
    if i == 1, title(ax, 'Velocity State Vectors'); legend(ax, 'Location', 'best'); end
    
    if i == 3, xlabel(ax, '$t$ [s]'); end
end

% --- Figura 4: Actuation Inputs ---
fig4 = figure('Name', 'Actuation Inputs', 'Color', 'w', 'Position', [670 50 1020 380]);

tlo3 = tiledlayout(fig4, 1, 3, 'TileSpacing', 'compact');

u_labels = {'$u_1$ ($\sim\theta$ cmd)', '$u_2$ ($\sim\phi$ cmd)', '$u_3$ ($\Delta T$ cmd)'};

for i = 1:3

    ax = nexttile(tlo3); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    
    plot(ax, t, u_nl(i, :), 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Nonlinear');
    
    plot(ax, t, u_lin(i, :), '--', 'Color', COL(i,:), 'LineWidth', 1.5, 'DisplayName', 'Linear');
    
    ylabel(ax, u_labels{i});
    
    xlabel(ax, '$t$ [s]');
    
    if i == 2, title(ax, 'Control Actuation Vectors'); end
    
    if i == 1, legend(ax, 'Location', 'best'); end
end