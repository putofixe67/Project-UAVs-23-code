%% ===============================================================
%  Lyapunov Design  –  Point 2
%  Runs class_1_4_LQR_Design first to get models(1–3) and all
%  workspace variables, then adds the Lyapunov controller as
%  models(4) and exports:
%    figures/lyapunov/  – 3D trajectory, inputs, position error,
%                         velocity error, error norm + animation
% ===============================================================

addpath("src\");

% Run LQR design (saves its own figure sets) and inherit workspace
class_1_4_LQR_Design();

close all;

% ================================================================
%  Lyapunov simulation  (uses N, dt, t, p_desired, v_desired,
%  a_desired, m, g, Kp, Kv, USE_V_DESIRED, USE_A_DESIRED from init)
% ================================================================
m = 29e-3;   % reset after class_1_4_LQR_Design may have changed it
g = 9.81;
d = 1e-4;

x_lya = zeros(N, 6);
u_lya = zeros(3, N);

for k = 1:N-1
    if USE_V_DESIRED
        xd = [p_desired(k,:)'; v_desired(k,:)'];
    else
        xd = [p_desired(k,:)'; zeros(3,1)];
    end

    xk = x_lya(k,:)';
    ek = xk - xd;
    ep = ek(1:3);
    ev = ek(4:6);

    if USE_A_DESIRED
        uk = lyapunovCtrl(ep, ev, Kp, Kv, a_desired(k,:)');
    else
        uk = lyapunovCtrl(ep, ev, Kp, Kv, 0);
    end

    u_lya(:,k) = uk;
    xdot = quad_dynamics_nonlinear(xk, m, g, uk);
    x_lya(k+1,:) = xk' + dt * xdot';
end

u_lya(:,N) = u_lya(:,N-1);

models(4).sysConsts = [m, g, d];
models(4).name      = 'Lyapunov';
models(4).x         = x_lya;
models(4).u         = u_lya * m;
models(4).color     = COL(4,:);

% ================================================================
%  Plots & export
% ================================================================
outDir = fullfile('figures', 'lyapunov');
plotResults(models, t, ref, ...
    ["Position", "Inputs", "PositionError", "VelocityError", "ErrorNorm"], outDir);
animateUAV(models, t, ref, outDir);
