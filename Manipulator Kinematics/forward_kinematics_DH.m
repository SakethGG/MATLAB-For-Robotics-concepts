% =========================================================================
% FORWARD KINEMATICS - Denavit-Hartenberg (DH) Convention
% =========================================================================
% Computes and visualises the forward kinematics of a serial manipulator
% using the standard DH convention.
%
% Each link i is described by 4 DH parameters:
%   a_i     : link length    (distance along x_i)
%   d_i     : link offset    (distance along z_{i-1})
%   alpha_i : link twist     (angle about x_i)
%   theta_i : joint angle    (angle about z_{i-1}) ← the variable for R joints
%
% The transform from frame i-1 to frame i is:
%   T_i = Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)
%
% DEFAULT EXAMPLE: 3-DOF planar robot (all revolute joints, a=[1 1 0.5])
% =========================================================================

fprintf('=== Forward Kinematics via DH Parameters ===\n\n');
fprintf('Default: 3-DOF planar arm  (press Enter to accept defaults)\n\n');

% --- DH Table: [a, d, alpha, theta] per row ---
%      a      d      alpha   theta (joint variable)
DH = [1.0,   0,     0,      0;    % Link 1
      1.0,   0,     0,      0;    % Link 2
      0.5,   0,     0,      0];   % Link 3

n_joints = size(DH, 1);
theta = zeros(1, n_joints);

fprintf('Enter joint angles (degrees) for %d joints:\n', n_joints);
for i = 1:n_joints
    theta(i) = deg2rad(input(sprintf('  θ%d: ', i)));
end

% --- Compute forward kinematics ---
T_total = eye(4);               % base frame
T_frames = zeros(4, 4, n_joints+1);
T_frames(:,:,1) = T_total;

for i = 1:n_joints
    a_i     = DH(i,1);
    d_i     = DH(i,2);
    alpha_i = DH(i,3);
    th_i    = theta(i) + DH(i,4);  % joint angle + DH offset

    % Standard DH transform
    T_i = dh_transform(a_i, d_i, alpha_i, th_i);
    T_total = T_total * T_i;
    T_frames(:,:,i+1) = T_total;
end

fprintf('\nEnd-Effector Position: [%.4f  %.4f  %.4f]\n', ...
    T_total(1,4), T_total(2,4), T_total(3,4));
fprintf('End-Effector Rotation Matrix:\n'); disp(T_total(1:3,1:3));

% --- Visualise ---
figure('Name','Forward Kinematics - DH','Color','w');
axis([-3 3 -3 3 -1 3]); grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Forward Kinematics  |  DH Convention');
view(30, 25);

% Plot links and frames
colors = {'b','r','g','m','c'};
origins = zeros(n_joints+1, 3);

for i = 1:n_joints+1
    T_i = T_frames(:,:,i);
    origins(i,:) = T_i(1:3,4)';
    plot_frame(T_i, colors{mod(i-1,5)+1}, sprintf('{%d}', i-1), 0.3);
end

% Draw links between joint origins
plot3(origins(:,1), origins(:,2), origins(:,3), 'k-o', ...
    'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% Highlight end-effector
ee = origins(end,:);
scatter3(ee(1), ee(2), ee(3), 100, 'r', 'filled');
text(ee(1)+0.05, ee(2)+0.05, ee(3)+0.05, 'EE', 'Color','r','FontWeight','bold');
hold off;

% =========================================================================
function T = dh_transform(a, d, alpha, theta)
    % Standard DH transformation matrix
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [ct,  -st*ca,  st*sa,  a*ct;
         st,   ct*ca, -ct*sa,  a*st;
          0,      sa,     ca,     d;
          0,       0,      0,     1];
end

function plot_frame(T, color, label, scale)
    R = T(1:3,1:3);
    p = T(1:3,4);
    ax = R * eye(3);
    for k = 1:3
        quiver3(p(1),p(2),p(3), ax(1,k)*scale, ax(2,k)*scale, ax(3,k)*scale, ...
            'Color',color,'LineWidth',1.5,'MaxHeadSize',2,'AutoScale','off');
    end
    text(p(1),p(2),p(3)+0.1, label, 'Color',color,'FontWeight','bold','FontSize',8);
end
