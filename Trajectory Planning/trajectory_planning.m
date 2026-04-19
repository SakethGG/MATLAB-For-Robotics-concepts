% =========================================================================
% TRAJECTORY PLANNING - Joint Space vs Cartesian Space
% =========================================================================
% Demonstrates three trajectory types between two configurations:
%
%   1. LINEAR joint-space interpolation (LSPB with blends)
%   2. CUBIC POLYNOMIAL joint-space trajectory
%   3. STRAIGHT-LINE Cartesian trajectory (via numerical IK at each point)
%
% Visualises position, velocity, and acceleration profiles alongside
% the animated 2-DOF arm motion.
% =========================================================================

fprintf('=== Trajectory Planning  |  2-DOF Planar Arm ===\n\n');

L = [1.0, 0.8];   % link lengths

% --- Start and goal joint angles ---
q_start = deg2rad([0,  0 ]);
q_goal  = deg2rad([60, -90]);

T   = 3.0;         % total motion time (seconds)
dt  = 0.02;        % time step
t   = 0:dt:T;
N   = length(t);

fprintf('Trajectory type:\n  1 - Linear (LSPB)\n  2 - Cubic Polynomial\n  3 - Cartesian Straight-Line\n');
choice = input('Choose [1/2/3, default 2]: ');
if isempty(choice), choice = 2; end

switch choice
    % -----------------------------------------------------------------------
    case 1   % Linear with parabolic blends (LSPB)
        tb = 0.3 * T;     % blend time
        q  = zeros(N, 2); qd = zeros(N, 2); qdd = zeros(N, 2);
        for j = 1:2
            [q(:,j), qd(:,j), qdd(:,j)] = lspb_traj(q_start(j), q_goal(j), t, tb);
        end
        traj_name = 'LSPB (Linear + Parabolic Blend)';

    % -----------------------------------------------------------------------
    case 2   % Cubic polynomial
        q  = zeros(N, 2); qd = zeros(N, 2); qdd = zeros(N, 2);
        for j = 1:2
            [q(:,j), qd(:,j), qdd(:,j)] = cubic_traj(q_start(j), q_goal(j), t, T);
        end
        traj_name = 'Cubic Polynomial';

    % -----------------------------------------------------------------------
    case 3   % Cartesian straight-line
        ee_start = fk_2dof(q_start, L);
        ee_goal  = fk_2dof(q_goal,  L);
        q  = zeros(N, 2); qd = zeros(N,2); qdd = zeros(N,2);
        q(1,:) = q_start;
        for k = 1:N
            s     = (k-1)/(N-1);
            ee_d  = (1-s)*ee_start + s*ee_goal;     % linear in Cartesian
            q(k,:) = ik_2dof(ee_d, L, q(max(k-1,1),:));  % numerical IK
        end
        % Numerical derivatives
        qd  = [diff(q,1,1)/dt; zeros(1,2)];
        qdd = [diff(qd,1,1)/dt; zeros(1,2)];
        traj_name = 'Cartesian Straight-Line';
end

% --- Compute Cartesian path ---
ee_path = zeros(N, 2);
for k = 1:N
    ee_path(k,:) = fk_2dof(q(k,:), L);
end

% --- Figure: Profiles + Animation ---
figure('Name',['Trajectory Planning: ', traj_name],'Color','w','Position',[50 50 1200 600]);

% Subplot layout
ax_arm = subplot(2,3,[1,4]); hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
lim = sum(L)+0.2;
axis(ax_arm,[-lim lim -lim lim]);
xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
title(ax_arm,['Arm Motion  |  ', traj_name]);

ax_pos  = subplot(2,3,2); hold on; grid on; title('Joint Positions');
ax_vel  = subplot(2,3,3); hold on; grid on; title('Joint Velocities');
ax_acc  = subplot(2,3,5); hold on; grid on; title('Joint Accelerations');
ax_cart = subplot(2,3,6); hold on; grid on; title('Cartesian Path');

% Plot profiles (static)
plot(ax_pos,  t, rad2deg(q),   'LineWidth', 2);
plot(ax_vel,  t, rad2deg(qd),  'LineWidth', 2);
plot(ax_acc,  t, rad2deg(qdd), 'LineWidth', 2);
xlabel(ax_pos,'t (s)');  ylabel(ax_pos,'Angle (°)');
xlabel(ax_vel,'t (s)');  ylabel(ax_vel,'Vel (°/s)');
xlabel(ax_acc,'t (s)');  ylabel(ax_acc,'Acc (°/s²)');
legend(ax_pos, 'θ1','θ2'); legend(ax_vel, 'θ̇1','θ̇2');
plot(ax_cart, ee_path(:,1), ee_path(:,2), 'b-', 'LineWidth', 2);
scatter(ax_cart, ee_path(1,1),ee_path(1,2),80,'g','filled');
scatter(ax_cart, ee_path(end,1),ee_path(end,2),80,'r','filled');
xlabel(ax_cart,'X (m)'); ylabel(ax_cart,'Y (m)'); axis(ax_cart,'equal');

% Animate
for k = 1:2:N
    joints = arm_joints(q(k,:), L);
    cla(ax_arm); hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
    axis(ax_arm,[-lim lim -lim lim]);
    plot(ax_arm, ee_path(1:k,1), ee_path(1:k,2), 'c-','LineWidth',1.5);
    plot(ax_arm, joints(:,1), joints(:,2), 'b-o','LineWidth',3,'MarkerSize',8,'MarkerFaceColor','b');
    scatter(ax_arm, ee_path(end,1),ee_path(end,2),80,'r','filled','Marker','pentagram');
    title(ax_arm, sprintf('%s  |  t=%.2fs', traj_name, t(k)));
    xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
    drawnow;
end

% =========================================================================
function [q, qd, qdd] = cubic_traj(q0, qf, t, T)
    % Zero velocity boundary conditions
    a0 = q0;  a1 = 0;
    a2 = 3*(qf-q0)/T^2;
    a3 = -2*(qf-q0)/T^3;
    q   = a0 + a1*t + a2*t.^2 + a3*t.^3;
    qd  = a1 + 2*a2*t + 3*a3*t.^2;
    qdd = 2*a2 + 6*a3*t;
end

function [q, qd, qdd] = lspb_traj(q0, qf, t, tb)
    T   = t(end);
    V   = (qf - q0) / (T - tb);     % cruise velocity
    q   = zeros(size(t));  qd = q;  qdd = q;
    for k = 1:length(t)
        tk = t(k);
        if tk <= tb
            q(k)   = q0 + V/(2*tb)*tk^2;
            qd(k)  = V/tb*tk;
            qdd(k) = V/tb;
        elseif tk <= T-tb
            q(k)   = q0 + V*(tk - tb/2);
            qd(k)  = V;
            qdd(k) = 0;
        else
            q(k)   = qf - V/(2*tb)*(T-tk)^2;
            qd(k)  = V/tb*(T-tk);
            qdd(k) = -V/tb;
        end
    end
end

function ee = fk_2dof(theta, L)
    x = L(1)*cos(theta(1)) + L(2)*cos(theta(1)+theta(2));
    y = L(1)*sin(theta(1)) + L(2)*sin(theta(1)+theta(2));
    ee = [x, y];
end

function theta = ik_2dof(ee, L, theta_prev)
    xd = ee(1); yd = ee(2);
    c2 = (xd^2+yd^2-L(1)^2-L(2)^2)/(2*L(1)*L(2));
    c2 = max(min(c2,1),-1);
    s2 = sqrt(1-c2^2);
    th2 = atan2(s2, c2);   % elbow-down
    th1 = atan2(yd,xd) - atan2(L(2)*s2, L(1)+L(2)*c2);
    theta = [th1, th2];
end

function joints = arm_joints(theta, L)
    joints = zeros(3, 2);
    joints(2,:) = joints(1,:) + L(1)*[cos(theta(1)), sin(theta(1))];
    joints(3,:) = joints(2,:) + L(2)*[cos(theta(1)+theta(2)), sin(theta(1)+theta(2))];
end
