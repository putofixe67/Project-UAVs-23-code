%% ===============================================
%  ---              Lyapunov Design            ---
% ================================================

addpath("src\");

% ============================
% Generate LQR results
% ============================
class_1_4_LQR_Design();

close all;

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

% ====================================
% --- Lyapunov Error Space Control ---
% ====================================

% States: [px py pz vx vy vz]
x_lya = zeros(N,6);
u_lya = zeros(3,N);

for k = 1:N-1

    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; 0; 0; 0];
    end

    xk = x_lya(k,:)';
    ek = xk - xd;

    ep = ek(1:3);   % position error
    ev = ek(4:6);   % velocity error
    u_ff = a_desired(k,:)';

    uk = lyapunovCtrl(ep, ev, Kp, Kv, 0);    
    if USE_A_DESIRED
        uk = lyapunovCtrl(ep, ev, Kp, Kv, u_ff);
    end
    
    u_lya(:,k) = uk;

    xdot = quad_dynamics_nonlinear(xk, m, g, uk);

    x_lya(k+1,:) = xk' + dt * xdot';
end

u_lya(:, N) = u_lya(:, N-1);

models(4).name = 'Lyapunov';
models(4).x    = x_lya;
models(4).u    = u_lya;
models(4).color = COL(4,:);

% ==========================================
% PLOTS
% ==========================================
plotResults(models, t, ref, ["Position", "Velocity", "Error", "ErrorNorm"]);