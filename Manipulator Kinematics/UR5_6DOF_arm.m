% =========================================================================
% UR5-LIKE 6-DOF ARM — Forward Kinematics & 3D Visualisation
% =========================================================================
% Implements forward kinematics for a UR5-inspired 6-DOF robot arm using
% the standard DH convention. The UR5 is one of the most widely used
% collaborative robots in industry and research.
%
% UR5 DH Parameters (Modified for Standard DH convention):
%   Joint  |   a (m)   |   d (m)   |  alpha (rad) | theta_offset
%     1    |   0.000   |  0.0892   |    pi/2      |     0
%     2    |  -0.425   |  0.0000   |    0         |     0
%     3    |  -0.3922  |  0.0000   |    0         |     0
%     4    |   0.000   |  0.1093   |    pi/2      |     0
%     5    |   0.000   |  0.0948   |   -pi/2      |     0
%     6    |   0.000   |  0.0825   |    0         |     0
%
% FEATURES:
%   - Full 3D animated arm with cylindrical links
%   - All 6 coordinate frames drawn
%   - Interactive: enter any 6 joint angles and see the result
%   - Displays full T matrix and EE pose
%   - Compares two configurations side by side
%
% Requires: No toolboxes
% =========================================================================

fprintf('============================================================\n');
fprintf('  UR5-Like 6-DOF Arm  |  Forward Kinematics\n');
fprintf('============================================================\n\n');

% -------------------------------------------------------------------------
%  UR5 DH Table  [a(m), d(m), alpha(rad), theta_offset(rad)]
% -------------------------------------------------------------------------
DH_UR5 = [
     0.000,   0.0892,   pi/2,   0;    % J1 — shoulder pan
    -0.425,   0.0000,   0,      0;    % J2 — shoulder lift
    -0.3922,  0.0000,   0,      0;    % J3 — elbow
     0.000,   0.1093,   pi/2,   0;    % J4 — wrist 1
     0.000,   0.0948,  -pi/2,   0;    % J5 — wrist 2
     0.000,   0.0825,   0,      0;    % J6 — wrist 3 (end-effector)
];

joint_names = {'Shoulder Pan','Shoulder Lift','Elbow',...
               'Wrist 1','Wrist 2','Wrist 3'};
n = size(DH_UR5, 1);

% UR5 joint limits [min, max] in degrees
j_lim = [-360,360; -360,360; -360,360; -360,360; -360,360; -360,360];

% -------------------------------------------------------------------------
%  Home position (all zeros = upright)
% -------------------------------------------------------------------------
home = zeros(1, n);

fprintf('UR5 Home Position (all zeros = upright arm):\n');
T_home = fk_chain(home, DH_UR5);
print_ee_pose(T_home{end});

fprintf('\nEnter joint angles (degrees) for Configuration 1:\n');
fprintf('  (Press Enter to use default: [0, -90, 90, -90, -90, 0])\n');
input_1 = input('  Joints [j1 j2 j3 j4 j5 j6]: ');
if isempty(input_1)
    input_1 = [0, -90, 90, -90, -90, 0];   % common "ready" pose
end

fprintf('\nEnter joint angles for Configuration 2 (for comparison):\n');
fprintf('  (Press Enter to use default: [45, -60, 80, -110, -90, 30])\n');
input_2 = input('  Joints [j1 j2 j3 j4 j5 j6]: ');
if isempty(input_2)
    input_2 = [45, -60, 80, -110, -90, 30];
end

theta1 = deg2rad(input_1);
theta2 = deg2rad(input_2);

% -------------------------------------------------------------------------
%  Compute FK for both configs
% -------------------------------------------------------------------------
frames1 = fk_chain(theta1, DH_UR5);
frames2 = fk_chain(theta2, DH_UR5);

fprintf('\n--- Configuration 1 ---\n'); print_ee_pose(frames1{end});
fprintf('\n--- Configuration 2 ---\n'); print_ee_pose(frames2{end});

% -------------------------------------------------------------------------
%  Figure: side-by-side 3D arms
% -------------------------------------------------------------------------
fig = figure('Name','UR5-Like 6-DOF Arm — Forward Kinematics',...
    'Color','w','Position',[50 50 1300 620]);

configs    = {theta1, theta2};
all_frames = {frames1, frames2};
titles     = {sprintf('Config 1: [%s]°', num2str(input_1,'%.0f ')), ...
              sprintf('Config 2: [%s]°', num2str(input_2,'%.0f '))};
ax_list    = gobjects(2,1);

for cfg = 1:2
    ax_list(cfg) = subplot(1,2,cfg);
    ax = ax_list(cfg);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    view(ax, 45, 25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    title(ax, titles{cfg}, 'FontSize',11,'FontWeight','bold');
    axis(ax,[-0.9 0.9 -0.9 0.9 -0.15 1.1]);

    frames = all_frames{cfg};
    draw_ur5(ax, frames);
end

sgtitle('UR5-Like 6-DOF Arm  |  Forward Kinematics  |  Standard DH',...
    'FontSize',13,'FontWeight','bold');

% -------------------------------------------------------------------------
%  Animate Configuration 1 → Configuration 2
% -------------------------------------------------------------------------
fprintf('\nAnimate transition from Config 1 → Config 2? (y/n) [y]: ');
ans_anim = input('','s');
if isempty(ans_anim) || strcmpi(ans_anim,'y')
    animate_transition(theta1, theta2, DH_UR5, ax_list(1));
end

% =========================================================================
function frames = fk_chain(theta, DH)
    % Returns cell array of all frame transforms {T_0, T_01, T_012, ...}
    n      = size(DH,1);
    frames = cell(n+1,1);
    frames{1} = eye(4);
    T = eye(4);
    for i = 1:n
        T = T * dh_transform(theta(i)+DH(i,4), DH(i,1), DH(i,2), DH(i,3));
        frames{i+1} = T;
    end
end

function T = dh_transform(theta, a, d, alpha)
    ct=cos(theta); st=sin(theta); ca=cos(alpha); sa=sin(alpha);
    T=[ct,-st*ca, st*sa, a*ct;
       st, ct*ca,-ct*sa, a*st;
        0,    sa,    ca,    d;
        0,     0,     0,    1];
end

function draw_ur5(ax, frames)
    % Link colours — one per joint
    link_cols = {[0.85 0.85 0.85],[0.9 0.5 0.1],[0.9 0.5 0.1],...
                 [0.6 0.6 0.9],[0.6 0.6 0.9],[0.6 0.6 0.9]};
    frame_scale = 0.05;

    for i = 1:length(frames)-1
        p1 = frames{i}(1:3,4);
        p2 = frames{i+1}(1:3,4);

        % Draw cylindrical link as thick line
        plot3(ax,[p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],...
            '-','Color',link_cols{i},'LineWidth',8);

        % Joint sphere
        scatter3(ax,p1(1),p1(2),p1(3),80,...
            'MarkerFaceColor',[0.3 0.3 0.3],'MarkerEdgeColor','k');
    end

    % End-effector sphere
    ee = frames{end}(1:3,4);
    scatter3(ax,ee(1),ee(2),ee(3),120,'r','filled','MarkerEdgeColor','k');

    % Draw coordinate frames (every other joint to avoid clutter)
    show_at = [1, 2, 4, 7];   % base, J1, J3, EE
    frame_colors = {'b','r','g'};
    frame_lw     = [2.0, 1.5, 1.5];
    frame_lbl    = {'{0}','{1}','{3}','EE'};
    for idx = 1:length(show_at)
        fi  = show_at(idx);
        T_f = frames{fi};
        p   = T_f(1:3,4);
        R   = T_f(1:3,1:3);
        for ax_idx = 1:3
            d_vec = R(:,ax_idx) * frame_scale;
            quiver3(ax,p(1),p(2),p(3),d_vec(1),d_vec(2),d_vec(3),...
                'Color',frame_colors{ax_idx},'LineWidth',frame_lw(ax_idx),...
                'MaxHeadSize',3,'AutoScale','off');
        end
        text(ax,p(1)+0.03,p(2)+0.03,p(3)+0.02,frame_lbl{idx},...
            'FontSize',9,'FontWeight','bold','Color',[0.2 0.2 0.2]);
    end

    % EE approach vector (tool z-axis)
    ee_z = frames{end}(1:3,3) * 0.08;
    quiver3(ax,ee(1),ee(2),ee(3),ee_z(1),ee_z(2),ee_z(3),...
        'r','LineWidth',2.5,'MaxHeadSize',3,'AutoScale','off');
    text(ax,ee(1)+ee_z(1)+0.02,ee(2)+ee_z(2),ee(3)+ee_z(3),'EE',...
        'Color','r','FontWeight','bold','FontSize',10);
end

function animate_transition(theta1, theta2, DH, ax)
    title(ax,'Animating transition...','FontSize',11,'FontWeight','bold');
    n_steps = 60;
    for k = 1:n_steps
        s      = k / n_steps;
        % Smooth interpolation using cosine (ease in/out)
        s_ease = (1 - cos(pi*s)) / 2;
        theta_k = theta1 + s_ease*(theta2 - theta1);
        frames_k = fk_chain(theta_k, DH);
        cla(ax); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
        axis(ax,[-0.9 0.9 -0.9 0.9 -0.15 1.1]);
        view(ax,45,25);
        xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
        draw_ur5(ax, frames_k);
        title(ax, sprintf('Transition  |  %.0f%%', s*100),...
            'FontSize',11,'FontWeight','bold');
        drawnow;
        pause(0.03);
    end
    title(ax,'Config 2 (final)','FontSize',11,'FontWeight','bold');
    fprintf('Animation complete.\n');
end

function print_ee_pose(T_ee)
    p = T_ee(1:3,4);
    R = T_ee(1:3,1:3);
    fprintf('  EE Position:  x=%.4f  y=%.4f  z=%.4f (m)\n',p(1),p(2),p(3));
    % Extract roll-pitch-yaw from rotation matrix
    pitch = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
    yaw   = atan2( R(2,1)/cos(pitch), R(1,1)/cos(pitch));
    roll  = atan2( R(3,2)/cos(pitch), R(3,3)/cos(pitch));
    fprintf('  EE Rotation:  roll=%.2f°  pitch=%.2f°  yaw=%.2f°\n',...
        rad2deg(roll),rad2deg(pitch),rad2deg(yaw));
end
