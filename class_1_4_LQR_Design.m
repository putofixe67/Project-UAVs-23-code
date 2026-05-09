clear; close all; clc;

%% ===============================================
%  ---              LQR Design            ---
% ================================================

function xdot = quad_dynamics_nonlinear(x, m, g, lambda, T)

    % --- State unpack ---
    p = x(1:3);
    v = x(4:6);

    % --- Angles unpack ---
    phi = lambda(1);
    theta = lambda(2);
    psi = lambda(3)

    % --- Rotation matrix (ZYX) ---
    cp = cos(phi); sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi); sy = sin(psi);

    R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
         sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
         -st,    ct*sp,             ct*cp];

    % --- Forces ---
    fT = R*[0;0;T];
    fg =[0;0;-m*g];
    
    % Input acceleration vector
    u = (fT + fg)/m;

    % Velocity Inertial
    vdot = u;

    % Position inertial
    pdot = v;


    % --- Full state derivative ---
    xdot = [pdot;
            vdot;
            ];
end


function xdot = quad_dynamics_linear(x, m, g, lambda, T)

    % --- State unpack ---
    p = x(1:3);
    v = x(4:6);
    
    % --- Angles unpack ---
    phi = lambda(1);
    theta = lambda(2);

    R = [1,  0, theta;
         0,  1, -phi;
         -theta, 0, 1];

    % --- Forces ---
    fT = R*[0;0;T];
    fg =[0;0;-m*g];
    
    % Input acceleration vector
    u = (fT + fg)/m;

    % Velocity Inertial
    vdot = u;

    % Position inertial
    pdot = v;


    % --- Full state derivative ---
    xdot = [pdot;
            vdot;
            ];
end

% =======================================
% crazyflyer drone variables consideration
% ---    and States Inicialization ---
% =======================================
m = 29e-3;
g = 9.81;
fg =[0;0;-m*g];
fT = R*[0;0;T];
R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
         sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
         -st,    ct*sp,             ct*cp];

% --- Variables declaration ---
syms px py pz vx vy vz phi theta psi wx wy wz real
syms Jx Jy Jz m l g real
syms T1 T2 T3 T4 cQ cT real
syms Cdx Cdy Cdz real

% --- States Inicialization ---
p      = [px; py; pz];
v      = [vx; vy; vz];

x = [p; v];

% --- Gravity and drag ---
% --- ZYX Rotation matrix R(phi, theta, psi) --
cp = cos(phi);   sp = sin(phi);
ct = cos(theta); st = sin(theta);
cy = cos(psi);   sy = sin(psi);

R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
     sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
     -st,    ct*sp,              ct*cp            ];

% --- Dynamics ---
pdot      = v;
vdot      = (sym(1)/m)*(fg + fT);

% =============================
%  --- State equation ---
% =============================
f = [pdot; vdot];
f = simplify(f);

% =============================
%      --- Jacobian ---
% =============================
%disp('Computing A = df/dx ...');
A = jacobian(f, x);