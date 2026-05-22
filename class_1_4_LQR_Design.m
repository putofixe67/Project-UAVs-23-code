clear; close all; clc;

%% ===============================================
%  ---              LQR Design            ---
% ================================================

addpath("src\");
init();

x_nl = zeros(N,6);
x_lin = zeros(N,6);

xLQR_ES = zeros(N,6);
x_error = zeros(N,6);

u_nl = zeros(3,N);
u_lin = zeros(3,N);

uLQR_ES = zeros(3,N);
u_error = zeros(3,N);

% ==================
% ---  Linear ---
% ==================
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

% ==================
% --- Nonlinear ---
% ==================
for k = 1:N-1
  
    % Reference for the desired state
    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; 0; 0; 0];
    end
    
    xk = x_nl(k,:)';
    
    % Control law
    uk = -K*(xk - xd);
    if USE_A_DESIRED
        uk = uk + a_desired(k,:)';
    end
    
    u_nl(:, k) = uk;

    xdot = quad_dynamics_nonlinear(xk, m, g, uk);
    x_nl(k+1,:) = xk' + dt*xdot';
end

u_nl(:, N) = u_nl(:, N-1);

% ===================================
% --- LQR Error Space Execution ---
% ===================================

A_error = [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_error = [zeros(3,3); eye(3)];
C_error = eye(6);

sys = ss(A_error, B_error, C_error, 0);

Q = eye(6);
R = eye(3);

K = lqr(sys, Q, R);

for k = 1:N-1

    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; 0; 0; 0];
    end

    xk = xLQR_ES(k,:)';

    ek = xk - xd;
    x_error(k,:) = ek';

    uk = -K * ek;

    if USE_A_DESIRED
        uk = uk + a_desired(k,:)';
    end

    uLQR_ES(:, k) = uk;

    xdot = quad_dynamics_nonlinear(xk, m, g, uk);
    xLQR_ES(k+1,:) = xk' + dt * xdot';
end

uLQR_ES(:, N) = uLQR_ES(:, N-1);

%% ==========================================
% Data structures to simplicity
models(1).name = 'LQR Nonlinear model';
models(1).x    = x_nl;
models(1).u    = u_nl;
models(1).color = COL(1,:);

models(2).name = 'LQR Nonlinear model ES';
models(2).x    = xLQR_ES;
models(2).u    = uLQR_ES;
models(2).color = COL(2,:);

models(3).name = 'LQR Linear model';
models(3).x    = x_lin;
models(3).u    = u_lin;
models(3).color = COL(3,:);

ref.p = p_desired;
ref.v = v_desired;
ref.a = a_desired;
% ==========================================

% ==========================================
% PLOTS
% ==========================================
plotResults(models, t, ref, ["Position"]);