function animateUAV(models, t, ref)
figure('Color','w');
ax = axes;
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
axis equal;
view(3);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('3D UAV Animation (Rotating Bodies)');

% ==========================================================
% UAV geometry (quadrotor cross)
% ==========================================================
arm = 0.25;
body = [
     0 0 0;
     arm 0 0;
     0 0 0;
    -arm 0 0;
     0 0 0;
     0 arm 0;
     0 0 0;
     0 -arm 0
];

nModels = numel(models);
hBody = gobjects(nModels,1);
hTraj = gobjects(nModels,1);

for k = 1:nModels
    hBody(k) = plot3(ax,0,0,0,'LineWidth',2,'Color',models(k).color,'DisplayName',models(k).name);
    hTraj(k) = plot3(ax,0,0,0,'--','Color',models(k).color,'HandleVisibility','off');
end

% Reference trajectory
plot3(ax,ref.p(:,1),ref.p(:,2),ref.p(:,3),'k--','LineWidth',1.5,'DisplayName','Reference');
legend(ax,'Location','best');

% ==========================================================
% Animation loop
% ==========================================================
for i = 1:5:length(t)
    for k = 1:nModels
        
        % ======================================================
        % State Position
        % ======================================================
        p = models(k).x(i,1:3);
        
        % Control input at current time-step (3x1 vector)
        uF = models(k).u(:,i);
        
        m = models(k).sysConsts(1);
        g = models(k).sysConsts(2);
        
        % ======================================================
        % ATTITUDE RECONSTRUCTION (LQR vs Lyapunov)
        % ======================================================
        if strcmpi(models(k).name, "Lyapunov")
            
            % For Lyapunov, uF is the total control vector acceleration
            a = uF + [0; 0; m*g];
            T = norm(a);
            b3 = a / max(T, 1e-6);
            
            % Compute exact Euler angles (in radians)
            theta = atan2(-b3(1), b3(3));
            phi   = atan2(b3(2), b3(3));
        else
            
            % For standard LQR models
            T = uF(3) + m*g;
            T = max(T, 1e-6);
            
            % Linearized small-angle approximations (assumed in radians)
            phi   =  uF(1) / T;  
            theta = -uF(2) / T; 
        end
        
        psi = 0; % Yaw assumed zero
        
        % Rotation matrices (using standard radians)
        Rz = [cos(psi) -sin(psi) 0;
              sin(psi)  cos(psi) 0;
              0         0        1];
          
        Ry = [cos(theta) 0 sin(theta);
              0          1 0;
             -sin(theta) 0 cos(theta)];
         
        Rx = [1 0 0;
              0 cos(phi) -sin(phi);
              0 sin(phi)  cos(phi)];
          
        R = Rz * Ry * Rx;
        
        % ======================================================
        % Rotate & Translate UAV body
        % ======================================================
        P = (R * body')';
        P = P + p;
        
        % ======================================================
        % Update Graphics Object Data
        % ======================================================
        set(hBody(k), 'XData', P(:,1), 'YData', P(:,2), 'ZData', P(:,3));
        set(hTraj(k), 'XData', models(k).x(1:i,1), 'YData', models(k).x(1:i,2), 'ZData', models(k).x(1:i,3));
    end
    
    drawnow limitrate;
    pause(0.06); % Adjusted down slightly since loop overhead takes time
end
end