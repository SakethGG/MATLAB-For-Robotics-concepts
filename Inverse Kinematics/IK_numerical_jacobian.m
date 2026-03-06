% =========================================================================
% INVERSE KINEMATICS - Numerical Method (Jacobian Pseudo-Inverse)
% =========================================================================
% Iteratively moves toward a target position using:
%   Δθ = J⁺ · Δx   where J⁺ = Jᵀ(JJᵀ)⁻¹  (Moore-Penrose pseudo-inverse)
%
% Works for any number of joints. Demonstrated on a 3-DOF planar arm.
% Shows convergence trajectory and joint angle evolution.
%
% INPUTS (prompted):  target (x, y),  link lengths,  initial joint angles
% =========================================================================

fprintf('=== Numerical IK via Jacobian Pseudo-Inverse ===\n\n');

L = [1.0, 0.8, 0.5];              % link lengths
n = length(L);
theta = zeros(1, n);               % initial joint angles (all zero)

fprintf('Link lengths: [%.1f  %.1f  %.1f]\n', L(1),L(2),L(3));
theta_init = input(sprintf('Enter initial joint angles (deg) [%d values, default 0s]: ', n));
if ~isempty(theta_init)
    theta = deg2rad(theta_init);
end

target = input('Enter target position [x y]: ');
xd = target(1);  yd = target(2);

% --- Solver settings ---
alpha    = 0.3;       % step size (learning rate)
tol      = 1e-3;      % convergence tolerance
max_iter = 500;

history_pos   = zeros(max_iter, 2);
history_theta = zeros(max_iter, n);

fprintf('\nRunning numerical IK...\n');

figure('Name','Numerical IK - Jacobian Pseudo-Inverse','Color','w');
subplot(1,2,1); hold on; grid on; axis equal;
lim = sum(L) + 0.5;
axis([-lim lim -lim lim]);
xlabel('X (m)'); ylabel('Y (m)');
title('End-Effector Trajectory');

for iter = 1:max_iter
    % --- FK: compute joint positions and EE ---
    [joints, ee] = fk_planar(theta, L);
    history_pos(iter,:)   = ee;
    history_theta(iter,:) = theta;

    err = [xd; yd] - ee(:);
    if norm(err) < tol
        fprintf('Converged in %d iterations  |  Error = %.6f\n', iter, norm(err));
        break;
    end

    % --- Jacobian (2 x n) for planar arm ---
    J = planar_jacobian(theta, L);

    % --- Pseudo-inverse update ---
    dtheta = alpha * (J' / (J*J')) * err;
    theta  = theta + dtheta';

    % --- Live plot every 5 iterations ---
    if mod(iter, 5) == 0
        subplot(1,2,1); cla; hold on; grid on; axis equal;
        axis([-lim lim -lim lim]);
        plot(history_pos(1:iter,1), history_pos(1:iter,2), 'c-', 'LineWidth', 1);
        draw_arm(joints, ee, 'b');
        scatter(xd, yd, 120, 'r', 'filled', 'Marker','pentagram');
        title(sprintf('Iteration %d  |  Error=%.4f', iter, norm(err)));
        xlabel('X (m)'); ylabel('Y (m)');
        drawnow;
    end
end

% --- Final state ---
[joints, ee] = fk_planar(theta, L);
subplot(1,2,1); cla; hold on; grid on; axis equal;
axis([-lim lim -lim lim]);
plot(history_pos(1:iter,1), history_pos(1:iter,2), 'c-','LineWidth',1.5);
draw_arm(joints, ee, 'b');
scatter(xd, yd, 120, 'r', 'filled','Marker','pentagram');
text(xd+0.05, yd+0.05, 'Target','Color','r','FontWeight','bold');
title(sprintf('Final  |  Error=%.5f  |  %d iters', norm([xd;yd]-ee(:)), iter));
xlabel('X (m)'); ylabel('Y (m)');

subplot(1,2,2); hold on; grid on;
colors2 = {'b','r','g'};
for j = 1:n
    plot(1:iter, rad2deg(history_theta(1:iter,j)), colors2{j}, 'LineWidth', 2);
end
xlabel('Iteration'); ylabel('Joint Angle (°)');
title('Joint Angle Convergence');
legend(arrayfun(@(i) sprintf('θ%d',i), 1:n,'UniformOutput',false));
sgtitle('Numerical Inverse Kinematics  |  Jacobian Pseudo-Inverse');

fprintf('\nSolution: ');
fprintf('θ%d=%.2f°  ', [1:n; rad2deg(theta)]);
fprintf('\n');

% =========================================================================
function [joints, ee] = fk_planar(theta, L)
    n  = length(L);
    joints = zeros(n+1, 2);
    cum_th = 0;
    for i = 1:n
        cum_th = cum_th + theta(i);
        joints(i+1,:) = joints(i,:) + L(i)*[cos(cum_th), sin(cum_th)];
    end
    ee = joints(end,:);
end

function J = planar_jacobian(theta, L)
    n  = length(L);
    J  = zeros(2, n);
    [joints, ee] = fk_planar(theta, L);
    for i = 1:n
        r  = ee - joints(i,:);         % vector from joint i to EE
        J(:,i) = [-r(2); r(1)];        % ẑ × r  for planar (z-axis rotation)
    end
end

function draw_arm(joints, ee, color)
    pts = [joints; ee];
    plot(pts(:,1), pts(:,2), [color,'-o'], 'LineWidth', 2.5, ...
        'MarkerSize', 7, 'MarkerFaceColor', color);
end
