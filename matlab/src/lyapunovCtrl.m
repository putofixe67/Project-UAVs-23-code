function uLyapunov = lyapunovCtrl(ep, ev, Kp, Kv, u_ff)
    % Lyapunov control law
    % V = e'*e  =>  V_dot = -ep'*Kp*ep - ev'*(Kv-I)*ev < 0 when Kv > I
    
    uLyapunov = -Kp * ep - Kv * ev + u_ff;
end
