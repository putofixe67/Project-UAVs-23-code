clear; close all; clc;

%% ===============================================================
%  LQR Design  –  Point 1.4
%  Generates three figure sets + two animations:
%    figures/lqr/feedforward/  – effect of v_d / a_d feedforward
%    figures/lqr/tuning/       – effect of Q/R tuning
%    figures/lqr/full/         – full results (Nonlinear, ES, Linear)
% ===============================================================

addpath("src");
init();

ref.p = p_desired;
ref.v = v_desired;
ref.a = a_desired;

%% ============================================================
%  PART 1 – FEEDFORWARD COMPARISON
%  LQR Nonlinear with 3 feedforward configurations
% ============================================================
ff_configs = [false false;   % No feedforward
              true  false;   % v_desired only
              true  true ];  % Full feedforward (v + a)
ff_names   = {'No Feedforward', 'v_{d} only', 'Full (v_{d} + a_{d})'};

for j = 1:3
    [xj, uj] = simulate_lqr(K_nl, @quad_dynamics_nonlinear, N, dt, ...
        p_desired, v_desired, a_desired, m, g, ff_configs(j,1), ff_configs(j,2));
    ff_models(j).sysConsts = [m, g, d];
    ff_models(j).name      = ff_names{j};
    ff_models(j).x         = xj;
    ff_models(j).u         = uj * m;
    ff_models(j).color     = COL(j,:);
end

outDir_ff = fullfile('figures', 'lqr', 'feedforward');
plotResults(ff_models, t, ref, ...
    ["Position", "PositionError", "VelocityError", "ErrorNorm"], outDir_ff);
animateUAV(ff_models, t, ref, outDir_ff);

%% ============================================================
%  PART 2 – Q/R TUNING COMPARISON
%  LQR Nonlinear, full feedforward: untuned vs tuned
% ============================================================
A_base    = [zeros(3,3), eye(3); zeros(3,3), -d*eye(3)];
B_base    = [zeros(3,3); eye(3)];
K_untuned = lqr(ss(A_base, B_base, eye(6), 0), 10*eye(6), 8.5*eye(3));

[x_un, u_un] = simulate_lqr(K_untuned, @quad_dynamics_nonlinear, N, dt, ...
    p_desired, v_desired, a_desired, m, g, true, true);
[x_tu, u_tu] = simulate_lqr(K_nl, @quad_dynamics_nonlinear, N, dt, ...
    p_desired, v_desired, a_desired, m, g, true, true);

tuning_models(1).sysConsts = [m, g, d];
tuning_models(1).name      = 'Untuned Q/R';
tuning_models(1).x         = x_un;
tuning_models(1).u         = u_un * m;
tuning_models(1).color     = COL(3,:);

tuning_models(2).sysConsts = [m, g, d];
tuning_models(2).name      = 'Tuned Q/R';
tuning_models(2).x         = x_tu;
tuning_models(2).u         = u_tu * m;
tuning_models(2).color     = COL(1,:);

outDir_tuning = fullfile('figures', 'lqr', 'tuning');
plotResults(tuning_models, t, ref, ...
    ["Position", "Inputs", "PositionError", "VelocityError"], outDir_tuning);

%% ============================================================
%  PART 3 – FULL LQR RESULTS (Tuned + Both FF)
%  Nonlinear, Error-Space Nonlinear, Linear
%  (This models struct is also used by class_2_Lyapunov_Design)
% ============================================================
[x_es,  u_es ] = simulate_lqr(K_LQR_ES, @quad_dynamics_nonlinear, N, dt, ...
    p_desired, v_desired, a_desired, m, g, true, true);
[x_lin, u_lin] = simulate_lqr(K_lin, @quad_dynamics_linear, N, dt, ...
    p_desired, v_desired, a_desired, m, g, true, true);

models(1).sysConsts = [m, g, d];
models(1).name      = 'LQR Nonlinear';
models(1).x         = x_tu;
models(1).u         = u_tu * m;
models(1).color     = COL(1,:);

models(2).sysConsts = [m, g, d];
models(2).name      = 'LQR Nonlinear ES';
models(2).x         = x_es;
models(2).u         = u_es * m;
models(2).color     = COL(2,:);

models(3).sysConsts = [m, g, d];
models(3).name      = 'LQR Linear';
models(3).x         = x_lin;
models(3).u         = u_lin * m;
models(3).color     = COL(3,:);

outDir_full = fullfile('figures', 'lqr', 'full');
plotResults(models, t, ref, ...
    ["Position", "Velocity", "Inputs", "PositionError", "VelocityError"], outDir_full);
animateUAV(models, t, ref, outDir_full);
