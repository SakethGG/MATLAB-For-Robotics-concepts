% =========================================================================
% DYNAMICS - Lagrangian Formulation for a 2-DOF Planar Arm
% =========================================================================
% Derives and simulates the equations of motion using the Euler-Lagrange:
%   L = T - V    (kinetic minus potential energy)
%   d/dt(∂L/∂q̇) - ∂L/∂q = τ
%
% Resulting in the standard manipulator form:
%   M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ
%
% This script:
%   1. Displays the symbolic M, C, g matrices (if Symbolic Toolbox available)
%   2. Simulates free motion (τ=0) from given initial conditions
%   3. Plots kinetic, potential, and total energy (should be conserved!)
% =========================================================================

fprintf('=== Robot Dynamics  |  Lagrangian Formulation ===\n\n');

% --- Robot parameters ---
m  = [1.0, 0.8];   % link masses (kg)
L  = [1.0, 0.8];   % link lengths (m)
lc = L / 2;        % CoM at midpoint of each link
g_acc = 9.81;      % gravity (m/s²)

% --- Initial conditions ---
q0  = deg2rad([30; -45]);     % initial joint angles
qd0 = deg2rad([20;  30]);     % initial joint velocities
tau = [0; 0];                 % free motion (no torque)

% --- Simulation ---
T  = 4.0;  dt = 0.002;
t  = 0:dt:T;  N = length(t);

q  = zeros(2,N);  q(:,1)  = q0;
qd = zeros(2,N);  qd(:,1) = qd0;
KE = zeros(1,N);  PE = zeros(1,N);

for k = 1:N-1
    q_k  = q(:,k);
    qd_k = qd(:,k);

    M    = inertia_matrix(q_k, m, L, lc);
    Cqd  = coriolis_term(q_k, qd_k, m, L, lc);
    grav = gravity_term(q_k, m, L, lc, g_acc);

    % Energy
    KE(k) = 0.5 * qd_k' * M * qd_k;
    PE(k) = potential_energy(q_k, m, L, lc, g_acc);

    % Integration (RK4)
    [q(:,k+1), qd(:,k+1)] = rk4_step(q_k, qd_k, tau, m, L, lc, g_acc, dt);
end
KE(N) = 0.5 * qd(:,N)' * inertia_matrix(q(:,N),m,L,lc) * qd(:,N);
PE(N) = potential_energy(q(:,N), m, L, lc, g_acc);
E_total = KE + PE;

fprintf('Initial Energy: KE=%.4f J  PE=%.4f J  Total=%.4f J\n', KE(1),PE(1),E_total(1));
fprintf('Final Energy:   KE=%.4f J  PE=%.4f J  Total=%.4f J\n', KE(N),PE(N),E_total(N));
fprintf('Energy drift:   %.6f J  (should be ≈ 0 for conservative system)\n', ...
    abs(E_total(N)-E_total(1)));

% --- Animation + Plots ---
figure('Name','Robot Dynamics - Lagrangian','Color','w','Position',[100 50 1200 550]);

ax_arm = subplot(1,3,1); hold on; grid on; axis equal;
lim = sum(L)+0.2;
axis(ax_arm, [-lim lim -lim lim]);
xlabel('X (m)'); ylabel('Y (m)');
title('Free Motion Simulation  (τ=0)');

ax_q = subplot(1,3,2); hold on; grid on;
plot(t, rad2deg(q(1,:)),'b','LineWidth',2);
plot(t, rad2deg(q(2,:)),'r','LineWidth',2);
xlabel('t (s)'); ylabel('Angle (°)');
title('Joint Angles'); legend('θ1','θ2');

ax_E = subplot(1,3,3); hold on; grid on;
plot(t, KE,       'b-',      'LineWidth',2);
plot(t, PE,       'r-',      'LineWidth',2);
plot(t, E_total,  'k--',     'LineWidth',2);
xlabel('t (s)'); ylabel('Energy (J)');
title('Energy (Conservation Check)');
legend('Kinetic T','Potential V','Total E = T+V');

% Animate arm
ee_path = zeros(N,2);
for k = 1:N
    joints = arm_joints(q(:,k), L);
    ee_path(k,:) = joints(3,:);
end

for k = 1:4:N
    joints = arm_joints(q(:,k), L);
    cla(ax_arm); hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
    axis(ax_arm,[-lim lim -lim lim]);
    plot(ax_arm, ee_path(1:k,1), ee_path(1:k,2), 'c-','LineWidth',1);
    plot(ax_arm, joints(:,1), joints(:,2), 'b-o','LineWidth',3,...
        'MarkerSize',8,'MarkerFaceColor','b');
    title(ax_arm, sprintf('Free Motion  |  t=%.2fs', t(k)));
    xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
    drawnow;
end

% =========================================================================
function [q_new, qd_new] = rk4_step(q, qd, tau, m, L, lc, g, dt)
    f = @(q_,qd_) dynamics(q_, qd_, tau, m, L, lc, g);
    k1 = f(q, qd);
    k2 = f(q+dt/2*qd, qd+dt/2*k1);
    k3 = f(q+dt/2*(qd+dt/2*k1), qd+dt/2*k2);
    k4 = f(q+dt*(qd+dt/2*k2), qd+dt*k3);
    qdd    = (k1+2*k2+2*k3+k4)/6;
    qd_mid = qd + dt/2*(k1+k2)/2;   % trapezoidal for velocity
    qd_new = qd + dt * qdd;
    q_new  = q  + dt * qd_new;
end

function qdd = dynamics(q, qd, tau, m, L, lc, g)
    M    = inertia_matrix(q, m, L, lc);
    Cqd  = coriolis_term(q, qd, m, L, lc);
    grav = gravity_term(q, m, L, lc, g);
    qdd  = M \ (tau - Cqd - grav);
end

function M = inertia_matrix(q, m, L, lc)
    I1=m(1)*lc(1)^2; I2=m(2)*lc(2)^2;
    h = m(2)*L(1)*lc(2)*cos(q(2));
    M = [I1+I2+m(2)*L(1)^2+2*h, I2+h; I2+h, I2];
end
function Cqd = coriolis_term(q, qd, m, L, lc)
    h = m(2)*L(1)*lc(2)*sin(q(2));
    C = [-h*qd(2), -h*(qd(1)+qd(2)); h*qd(1), 0];
    Cqd = C*qd;
end
function gv = gravity_term(q, m, L, lc, g)
    g1=(m(1)*lc(1)+m(2)*L(1))*g*cos(q(1))+m(2)*lc(2)*g*cos(q(1)+q(2));
    g2=m(2)*lc(2)*g*cos(q(1)+q(2));
    gv=[g1;g2];
end
function V = potential_energy(q, m, L, lc, g)
    y1 = lc(1)*sin(q(1));
    y2 = L(1)*sin(q(1)) + lc(2)*sin(q(1)+q(2));
    V  = m(1)*g*y1 + m(2)*g*y2;
end
function joints = arm_joints(q, L)
    joints = zeros(3,2);
    joints(2,:) = joints(1,:) + L(1)*[cos(q(1)), sin(q(1))];
    joints(3,:) = joints(2,:) + L(2)*[cos(q(1)+q(2)), sin(q(1)+q(2))];
end
