% =========================================================================
% GEOMETRIC vs ANALYTICAL JACOBIAN — 3D Serial Arm
% =========================================================================
% Two methods exist to compute the Jacobian. They give the SAME result for
% position rows but differ in how orientation rows are derived.
%
% GEOMETRIC JACOBIAN (J_g):
%   Built column-by-column from the robot's geometry.
%   For a revolute joint i, the i-th column is:
%       J_g(:,i) = [ z_{i-1} × (p_e - p_{i-1}) ]   ← linear velocity part
%                  [          z_{i-1}            ]   ← angular velocity part
%   where z_{i-1} is the z-axis of frame i-1, p_{i-1} is its origin,
%   and p_e is the end-effector position.
%
% ANALYTICAL JACOBIAN (J_a):
%   Derived by differentiating the forward kinematics equations:
%       J_a = ∂x/∂q   where x is the task-space vector [px,py,pz,φ,θ,ψ]
%   The orientation part depends on the chosen representation (e.g. RPY).
%   J_a = T_inv(φ,θ,ψ) · J_g  where T_inv maps ω → φ̇,θ̇,ψ̇
%
% KEY INSIGHT:
%   - Position rows (top 3) are IDENTICAL for both Jacobians
%   - Orientation rows (bottom 3) differ when using RPY/Euler angles
%   - At singularities of the representation (e.g. θ=±90° for RPY),
%     J_a becomes undefined even though J_g is fine → representational singularity
%
% Robot: 3-DOF RRR arm in 3D (shoulder-elbow-wrist)
%   Joint 1: rotates about world Z  (pan)
%   Joint 2: rotates about local Y  (shoulder lift)
%   Joint 3: rotates about local Y  (elbow)
%
% No toolboxes required.
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Geometric vs Analytical Jacobian  |  3D RRR Arm\n');
fprintf('============================================================\n\n');

% --- Robot DH parameters [a, d, alpha, theta_offset] ---
%        a      d      alpha   offset
DH = [0.000,  0.000,  pi/2,   0;    % Joint 1: pan   (about Z)
      0.500,  0.000,  0,      0;    % Joint 2: lift  (about Y)
      0.400,  0.000,  0,      0];   % Joint 3: elbow (about Y)

n = size(DH,1);
L = DH(:,1)';

fprintf('Robot: 3-DOF RRR arm\n');
fprintf('Link lengths: [%.2f, %.2f, %.2f] m\n\n', L(1),L(2),L(3));

% --- Prompt for joint angles ---
fprintf('Enter joint angles to evaluate Jacobians at:\n');
fprintf('  (Press Enter for default: [30, -45, 60] degrees)\n');
q_in = input('  [q1 q2 q3] in degrees: ');
if isempty(q_in), q_in = [30, -45, 60]; end
q = deg2rad(q_in);

fprintf('\nComputing at q = [%.1f°, %.1f°, %.1f°]\n\n', q_in(1),q_in(2),q_in(3));

% =========================================================================
%  STEP 1: Forward Kinematics — get all frame transforms
% =========================================================================
[frames, T_total] = fk_chain(q, DH);

p_e = T_total(1:3,4);                   % EE position
R_e = T_total(1:3,1:3);                 % EE rotation

% Extract RPY from rotation matrix
[rpy] = rot2rpy(R_e);                   % [roll, pitch, yaw]

fprintf('FK Result:\n');
fprintf('  EE position:  [%.4f, %.4f, %.4f] m\n', p_e(1),p_e(2),p_e(3));
fprintf('  EE RPY:       [%.2f°, %.2f°, %.2f°]\n\n', ...
    rad2deg(rpy(1)),rad2deg(rpy(2)),rad2deg(rpy(3)));

% =========================================================================
%  STEP 2: GEOMETRIC JACOBIAN
%  J_g(:,i) = [z_{i-1} × (p_e - p_{i-1}); z_{i-1}]  for revolute joints
% =========================================================================
J_g = zeros(6, n);

for i = 1:n
    T_prev  = frames{i};               % transform up to frame i-1
    z_prev  = T_prev(1:3,3);           % z-axis of frame i-1
    p_prev  = T_prev(1:3,4);           % origin of frame i-1

    % Linear velocity component: z × (p_e - p_{i-1})
    J_g(1:3, i) = cross(z_prev, p_e - p_prev);

    % Angular velocity component: z_{i-1}
    J_g(4:6, i) = z_prev;
end

% =========================================================================
%  STEP 3: ANALYTICAL JACOBIAN (RPY representation)
%  Relates joint velocities to [ṗ_x, ṗ_y, ṗ_z, ṙoll, ṗitch, ẏaw]
%
%  The angular velocity ω and RPY rates φ̇ are related by:
%      ω = T_rpy(φ,θ,ψ) · [φ̇, θ̇, ψ̇]ᵀ
%  So:  J_a = [I_{3x3},        0      ] · J_g
%             [   0,      T_rpy^{-1}  ]
%
%  T_rpy for ZYX convention (yaw-pitch-roll):
%      T_rpy = [  cψ·cθ,  -sψ,   0  ]
%               [  sψ·cθ,   cψ,   0  ]
%               [   -sθ,    0,    1  ]
% =========================================================================
phi   = rpy(1);   % roll
theta = rpy(2);   % pitch
psi   = rpy(3);   % yaw

% T_rpy maps RPY rates to angular velocity
T_rpy = [cos(psi)*cos(theta), -sin(psi), 0;
          sin(psi)*cos(theta),  cos(psi), 0;
         -sin(theta),           0,        1];

% Check for representational singularity (pitch = ±90°)
rep_singular = abs(cos(theta)) < 0.01;

if rep_singular
    fprintf('⚠ REPRESENTATIONAL SINGULARITY DETECTED\n');
    fprintf('  Pitch angle θ = %.2f° ≈ ±90°\n', rad2deg(theta));
    fprintf('  T_rpy is not invertible — J_a is undefined here!\n');
    fprintf('  (J_g is still perfectly valid at this configuration)\n\n');
    J_a = NaN(6,n);
else
    % Analytical Jacobian: transform bottom 3 rows of J_g
    T_inv = inv(T_rpy);
    J_a   = J_g;
    J_a(4:6,:) = T_inv * J_g(4:6,:);
end

% =========================================================================
%  STEP 4: Print and compare
% =========================================================================
fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  GEOMETRIC JACOBIAN  J_g  (6×%d)\n', n);
fprintf('═══════════════════════════════════════════════════════\n');
print_jacobian(J_g, {'vx','vy','vz','ωx','ωy','ωz'});

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('  ANALYTICAL JACOBIAN  J_a  (6×%d)  — RPY representation\n', n);
fprintf('═══════════════════════════════════════════════════════\n');
print_jacobian(J_a, {'vx','vy','vz','φ̇ ','θ̇ ','ψ̇ '});

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('  COMPARISON — Position rows (top 3) should be IDENTICAL\n');
fprintf('═══════════════════════════════════════════════════════\n');
if ~any(isnan(J_a(:)))
    diff_pos = norm(J_g(1:3,:) - J_a(1:3,:));
    diff_ori = norm(J_g(4:6,:) - J_a(4:6,:));
    fprintf('  ||J_g_pos - J_a_pos|| = %.2e  (should be ≈ 0)\n', diff_pos);
    fprintf('  ||J_g_ori - J_a_ori|| = %.4f  (differs — different representation)\n', diff_ori);
end

% Manipulability from both
w_g = sqrt(max(det(J_g(1:3,:)*J_g(1:3,:)'),0));
fprintf('\n  Manipulability (position only):\n');
fprintf('    w = sqrt(det(Jv·Jvᵀ)) = %.6f\n', w_g);

% =========================================================================
%  STEP 5: Visualisation — 3-panel figure
% =========================================================================
fig = figure('Name','Geometric vs Analytical Jacobian','Color','w',...
    'Position',[50 50 1350 560]);

% --- Panel 1: 3D arm with frames and velocity arrows ---
ax3d = subplot(1,3,1);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
view(ax3d,45,25); axis(ax3d,[-0.5 1.1 -0.8 0.8 -0.2 1.1]);
xlabel(ax3d,'X'); ylabel(ax3d,'Y'); zlabel(ax3d,'Z');
title(ax3d,'3D Arm + Frame z-axes','FontSize',11,'FontWeight','bold');
draw_3d_arm(ax3d, frames, T_total, DH);
draw_z_axes(ax3d, frames, n);

% Show a sample EE velocity and the required joint velocities
v_ee_sample = [0.1; 0; 0; 0; 0; 0];    % unit motion in X
if ~any(isnan(J_g(:)))
    dq_g = pinv(J_g) * v_ee_sample;
    draw_velocity_arrow(ax3d, p_e, v_ee_sample(1:3)*0.25, 'r', 'v_{EE}');
end

% --- Panel 2: Geometric Jacobian heatmap ---
ax_jg = subplot(1,3,2);
plot_jacobian_heatmap(ax_jg, J_g, ...
    {'q_1','q_2','q_3'}, {'v_x','v_y','v_z','\omega_x','\omega_y','\omega_z'},...
    'Geometric Jacobian  J_g');

% --- Panel 3: Analytical Jacobian heatmap ---
ax_ja = subplot(1,3,3);
if ~any(isnan(J_a(:)))
    plot_jacobian_heatmap(ax_ja, J_a, ...
        {'q_1','q_2','q_3'}, {'v_x','v_y','v_z','\phi','\ \theta','\psi'},...
        'Analytical Jacobian  J_a  (RPY)');
else
    axis(ax_ja,'off');
    text(ax_ja,0.5,0.5,...
        sprintf('J_a UNDEFINED\nat this configuration\n(Representational Singularity)\nPitch = %.1f°',rad2deg(theta)),...
        'Units','normalized','HorizontalAlignment','center',...
        'FontSize',13,'FontWeight','bold','Color',[0.8 0.1 0.1]);
    title(ax_ja,'Analytical Jacobian  J_a  (RPY)','FontSize',11,'FontWeight','bold');
end

% Annotation: key difference
annotation(fig,'textbox',[0.01 0.01 0.98 0.06],...
    'String',['KEY: Position rows (top 3) are identical in both Jacobians.  ',...
    'Orientation rows differ — J_g gives angular velocity ω directly;  ',...
    'J_a gives RPY rates (φ̇,θ̇,ψ̇) which depend on current orientation.  ',...
    'J_a becomes singular when cos(pitch)=0 (±90°) — a REPRESENTATIONAL singularity, not a robot singularity!'],...
    'FontSize',9,'EdgeColor',[0.7 0.7 0.7],'BackgroundColor',[0.98 0.98 0.90],...
    'Interpreter','none');

sgtitle(sprintf('Geometric vs Analytical Jacobian  |  q=[%.1f°, %.1f°, %.1f°]',...
    q_in(1),q_in(2),q_in(3)),'FontSize',13,'FontWeight','bold');

% =========================================================================
%  HELPER FUNCTIONS
% =========================================================================
function [frames, T_total] = fk_chain(q, DH)
    n = size(DH,1);
    frames    = cell(n+1,1);
    frames{1} = eye(4);
    T = eye(4);
    for i = 1:n
        Ti = dh_mat(q(i)+DH(i,4), DH(i,1), DH(i,2), DH(i,3));
        T  = T * Ti;
        frames{i+1} = T;
    end
    T_total = T;
end

function T = dh_mat(th,a,d,al)
    ct=cos(th);st=sin(th);ca=cos(al);sa=sin(al);
    T=[ct,-st*ca,st*sa,a*ct; st,ct*ca,-ct*sa,a*st; 0,sa,ca,d; 0,0,0,1];
end

function rpy = rot2rpy(R)
    pitch = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
    yaw   = atan2(R(2,1)/cos(pitch), R(1,1)/cos(pitch));
    roll  = atan2(R(3,2)/cos(pitch), R(3,3)/cos(pitch));
    rpy   = [roll; pitch; yaw];
end

function print_jacobian(J, row_labels)
    fprintf('  Row\\Col  |   q1      q2      q3\n');
    fprintf('  ---------+------------------------\n');
    for r = 1:6
        fprintf('  %-6s   |', row_labels{r});
        for c = 1:size(J,2)
            if isnan(J(r,c))
                fprintf('  %6s', '  NaN ');
            else
                fprintf('  %+6.3f', J(r,c));
            end
        end
        fprintf('\n');
    end
end

function draw_3d_arm(ax, frames, T_total, DH)
    n  = length(frames)-1;
    sc = 0.12;
    cols = {'r','g','b'};
    lbl  = {'x','y','z'};
    for i = 1:n+1
        p = frames{i}(1:3,4);
        R = frames{i}(1:3,1:3);
        if i <= n+1
            for k=1:3
                quiver3(ax,p(1),p(2),p(3),R(1,k)*sc,R(2,k)*sc,R(3,k)*sc,...
                    'Color',cols{k},'LineWidth',1.5,'MaxHeadSize',3,'AutoScale','off');
            end
        end
        if i > 1
            p_prev = frames{i-1}(1:3,4);
            plot3(ax,[p_prev(1),p(1)],[p_prev(2),p(2)],[p_prev(3),p(3)],...
                'k-','LineWidth',6);
            scatter3(ax,p_prev(1),p_prev(2),p_prev(3),60,'k','filled');
        end
    end
    ee = T_total(1:3,4);
    scatter3(ax,ee(1),ee(2),ee(3),100,'r','filled');
    text(ax,ee(1)+0.04,ee(2),ee(3)+0.05,'EE','Color','r','FontWeight','bold');
end

function draw_z_axes(ax, frames, n)
    for i = 1:n
        p = frames{i}(1:3,4);
        z = frames{i}(1:3,3);
        quiver3(ax,p(1),p(2),p(3),z(1)*0.18,z(2)*0.18,z(3)*0.18,...
            'Color',[0.6 0 0.8],'LineWidth',2.5,'MaxHeadSize',3,'AutoScale','off');
        text(ax,p(1)+z(1)*0.22,p(2)+z(2)*0.22,p(3)+z(3)*0.22,...
            sprintf('z_{%d}',i-1),'Color',[0.6 0 0.8],'FontSize',9,'FontWeight','bold');
    end
end

function draw_velocity_arrow(ax, p, v, color, lbl)
    quiver3(ax,p(1),p(2),p(3),v(1),v(2),v(3),...
        'Color',color,'LineWidth',3,'MaxHeadSize',3,'AutoScale','off');
    text(ax,p(1)+v(1)+0.02,p(2)+v(2),p(3)+v(3)+0.02,lbl,...
        'Color',color,'FontWeight','bold','FontSize',10);
end

function plot_jacobian_heatmap(ax, J, col_labels, row_labels, ttl)
    imagesc(ax, J);
    colormap(ax, redblue_colormap());
    clim(ax, [-max(abs(J(:)))-0.01, max(abs(J(:)))+0.01]);
    colorbar(ax);
    set(ax,'XTick',1:size(J,2),'XTickLabel',col_labels,'FontSize',10);
    set(ax,'YTick',1:size(J,1),'YTickLabel',row_labels,'FontSize',10);
    title(ax,ttl,'FontSize',11,'FontWeight','bold');
    xlabel(ax,'Joint'); ylabel(ax,'Task-space component');
    % Annotate cell values
    for r=1:size(J,1)
        for c=1:size(J,2)
            if ~isnan(J(r,c))
                text(ax,c,r,sprintf('%.3f',J(r,c)),...
                    'HorizontalAlignment','center','FontSize',8.5,...
                    'Color','k','FontWeight','bold');
            end
        end
    end
    % Draw separator line between position and orientation rows
    hold(ax,'on');
    plot(ax,[0.5, size(J,2)+0.5],[3.5 3.5],'w-','LineWidth',2.5);
    text(ax,size(J,2)+0.55, 1.5,'Position','FontSize',8,'Color',[0.2 0.2 0.8],...
        'Rotation',270,'FontWeight','bold','Clipping','off');
    text(ax,size(J,2)+0.55, 4.8,'Orientation','FontSize',8,'Color',[0.2 0.6 0.2],...
        'Rotation',270,'FontWeight','bold','Clipping','off');
end

function cmap = redblue_colormap()
    n = 64;
    r = [linspace(0.1,1,n/2), ones(1,n/2)];
    b = [ones(1,n/2), linspace(1,0.1,n/2)];
    g = [linspace(0.3,1,n/2), linspace(1,0.3,n/2)];
    cmap = [r(:), g(:), b(:)];
end
