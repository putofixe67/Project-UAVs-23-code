function [x, u] = simulate_lqr(K, dynamics_fn, N, dt, p_des, v_des, a_des, m, g, use_v, use_a)
% Runs one LQR simulation. dynamics_fn: @quad_dynamics_nonlinear or @quad_dynamics_linear
x = zeros(N, 6);
u = zeros(3, N);
for k = 1:N-1
    if use_v
        xd = [p_des(k,:)'; v_des(k,:)'];
    else
        xd = [p_des(k,:)'; zeros(3,1)];
    end
    uk = -K * (x(k,:)' - xd);
    if use_a
        uk = uk + a_des(k,:)';
    end
    u(:,k) = uk;
    xdot = dynamics_fn(x(k,:)', m, g, uk);
    x(k+1,:) = x(k,:) + dt * xdot';
end
u(:,N) = u(:,N-1);
end
