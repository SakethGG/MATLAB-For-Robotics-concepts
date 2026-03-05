% =========================================================================
% WORKSPACE VISUALISATION — Reachable & Dexterous Workspace
% =========================================================================
% Answers the fundamental question: "Where can this robot reach?"
%
% Two workspaces are shown:
%   REACHABLE workspace  — all positions the EE can reach (any orientation)
%   DEXTEROUS workspace  — positions reachable in ALL orientations
%                          (subset of reachable, shown in darker colour)
%
% Method: Monte-Carlo sweep — randomly sample joint angles within limits,
% compute FK for each sample, collect EE positions.
%
% TWO VIEWS shown side by side:
%   LEFT  — 3D point cloud (full volume, rotatable)
%   RIGHT — 2D cross-section in the XZ plane (classic textbook view)
%
% Robot: 3-DOF planar-like arm with joint limits (adjustable)
%
% No toolboxes required.
% =========================================================================

fprintf('============================================================\n');
fprintf('  Workspace Visualisation  |  3-DOF Serial Arm\n');
fprintf('============================================================\n\n');

% -------------------------------------------------------------------------
%  Robot DH Parameters  (Standard DH: [a, d, alpha, theta_offset])
%  Default: elbow-style arm that sweeps in 3D
% -------------------------------------------------------------------------
%        a      d      alpha        theta_offset
DH = [0.00,  0.00,  pi/2,    0;    % Joint 1: shoulder (rotates about world Z)
      0.50,  0.00,  0,       0;    % Joint 2: upper arm  (L=0.5m)
      0.40,  0.00,  0,       0];   % Joint 3: forearm    (L=0.4m)

n = size(DH, 1);

% Joint limits [min, max] in degrees
j_limits_deg = [-180, 180;    % Joint 1
                 -150, 150;   % Joint 2
                 -150, 150];  % Joint 3

j_limits = deg2rad(j_limits_deg);

fprintf('Robot: %d-DOF arm\n', n);
fprintf('Link lengths: a = [%.2f, %.2f, %.2f] m\n', DH(1,1),DH(2,1),DH(3,1));
fprintf('\nJoint limits:\n');
for i = 1:n
    fprintf('  Joint %d: [%.0f°, %.0f°]\n', i, j_limits_deg(i,1), j_limits_deg(i,2));
end

N_samples = input(sprintf('\nNumber of random samples (default 50000): '));
if isempty(N_samples), N_samples = 50000; end

fprintf('\nSampling workspace...');

% -------------------------------------------------------------------------
%  Monte-Carlo sampling
% -------------------------------------------------------------------------
ee_reach  = zeros(N_samples, 3);   % reachable workspace points
ee_dext   = zeros(N_samples, 3);   % dexterous workspace points
n_dext    = 0;

% For dexterous: check if EE can be reached at 8 different wrist orientations
% (simplified: check 3 different theta_3 extreme values)
dext_configs = [j_limits(3,1), 0, j_limits(3,2)];

for k = 1:N_samples
    % Random joint angles within limits
    theta = zeros(1, n);
    for j = 1:n
        theta(j) = j_limits(j,1) + rand()*(j_limits(j,2)-j_limits(j,1));
    end

    % FK — get EE position
    T = fk_dh(theta, DH);
    ee_reach(k,:) = T(1:3,4)';

    % Dexterous check: reachable with joint 3 at both extremes?
    is_dext = true;
    for dc = dext_configs
        theta_check    = theta;
        theta_check(3) = dc;
        T_check = fk_dh(theta_check, DH);
        % Must be within 0.05m of original EE position
        if norm(T_check(1:3,4)' - ee_reach(k,:)) > 0.15
            is_dext = false;
            break;
        end
    end
    if is_dext
        n_dext = n_dext + 1;
        ee_dext(n_dext,:) = ee_reach(k,:);
    end
end
ee_dext = ee_dext(1:n_dext,:);

fprintf(' done.\n');
fprintf('Reachable samples:  %d\n', N_samples);
fprintf('Dexterous samples:  %d  (%.1f%%)\n', n_dext, 100*n_dext/N_samples);

max_reach = sum(DH(:,1));
fprintf('\nTheoretical max reach: %.3f m\n', max_reach);
fprintf('Actual max EE radius:  %.3f m\n', max(vecnorm(ee_reach(:,1:2)', 1)));

% -------------------------------------------------------------------------
%  Plotting
% -------------------------------------------------------------------------
fig = figure('Name','Workspace Visualisation','Color','w',...
    'Position',[60 60 1200 560]);

% --- LEFT: 3D Point Cloud ---
ax3d = subplot(1,2,1);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
view(ax3d, 35, 25);
xlabel(ax3d,'X (m)'); ylabel(ax3d,'Y (m)'); zlabel(ax3d,'Z (m)');
title(ax3d, '3D Workspace (Rotatable)','FontSize',12,'FontWeight','bold');

% Plot reachable (thin, transparent)
scatter3(ax3d, ee_reach(:,1), ee_reach(:,2), ee_reach(:,3),...
    1, [0.6 0.8 1.0], 'filled', 'MarkerFaceAlpha', 0.15);

% Plot dexterous (bolder)
if n_dext > 100
    scatter3(ax3d, ee_dext(:,1), ee_dext(:,2), ee_dext(:,3),...
        3, [0.1 0.3 0.8], 'filled', 'MarkerFaceAlpha', 0.4);
end

% Base frame
plot3(ax3d, 0,0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');
quiver3(ax3d,0,0,0, 0.15,0,0,'r','LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
quiver3(ax3d,0,0,0, 0,0.15,0,'g','LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
quiver3(ax3d,0,0,0, 0,0,0.15,'b','LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
text(ax3d,0.17,0,0,'X','Color','r','FontWeight','bold');
text(ax3d,0,0.17,0,'Y','Color','g','FontWeight','bold');
text(ax3d,0,0,0.17,'Z','Color','b','FontWeight','bold');

legend(ax3d, 'Reachable','Dexterous','Base',...
    'Location','best','FontSize',9);

% --- RIGHT: 2D Cross-section (XZ plane, i.e. Y=0 slice) ---
ax2d = subplot(1,2,2);
hold(ax2d,'on'); grid(ax2d,'on'); axis(ax2d,'equal');
xlabel(ax2d,'X (m)'); ylabel(ax2d,'Z (m)');
title(ax2d,'2D Cross-Section  (XZ plane)','FontSize',12,'FontWeight','bold');

% Filter points near Y=0 plane (±5% of max reach)
tol = max_reach * 0.05;
near_plane = abs(ee_reach(:,2)) < tol;
ee_2d = ee_reach(near_plane, [1,3]);   % X and Z columns

scatter(ax2d, ee_2d(:,1), ee_2d(:,2), 2, [0.5 0.75 1.0], 'filled',...
    'MarkerFaceAlpha', 0.3);

% Dexterous cross-section
if n_dext > 0
    near_p_d = abs(ee_dext(:,2)) < tol;
    ed_2d    = ee_dext(near_p_d, [1,3]);
    if ~isempty(ed_2d)
        scatter(ax2d, ed_2d(:,1), ed_2d(:,2), 5, [0.1 0.3 0.8], 'filled',...
            'MarkerFaceAlpha', 0.5);
    end
end

% Draw theoretical boundary circles
th_circ = linspace(0,2*pi,300);
plot(ax2d, max_reach*cos(th_circ), max_reach*sin(th_circ),...
    'k--','LineWidth',1.5,'DisplayName','Max reach boundary');
inner = abs(DH(2,1) - DH(3,1));
if inner > 0.01
    plot(ax2d, inner*cos(th_circ), inner*sin(th_circ),...
        'k:','LineWidth',1,'DisplayName','Min reach boundary');
end

% Base point
plot(ax2d, 0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');

legend(ax2d, 'Reachable','Dexterous','Max boundary','Min boundary','Base',...
    'Location','best','FontSize',9);

% Superimpose a sample arm configuration
theta_sample = deg2rad([30, 60, -90]);
draw_arm_2d(ax2d, theta_sample, DH);

sgtitle(sprintf('Workspace  |  %d-DOF Arm  |  %d samples',...
    n, N_samples),'FontSize',13,'FontWeight','bold');

fprintf('\nFigure is interactive — left plot is rotatable (click+drag).\n');

% =========================================================================
function T_total = fk_dh(theta, DH)
    n       = size(DH,1);
    T_total = eye(4);
    for i = 1:n
        a_i  = DH(i,1);
        d_i  = DH(i,2);
        al_i = DH(i,3);
        th_i = theta(i) + DH(i,4);
        ct = cos(th_i); st = sin(th_i);
        ca = cos(al_i); sa = sin(al_i);
        T_i = [ct, -st*ca,  st*sa, a_i*ct;
               st,  ct*ca, -ct*sa, a_i*st;
                0,     sa,     ca,    d_i;
                0,      0,      0,      1];
        T_total = T_total * T_i;
    end
end

function draw_arm_2d(ax, theta, DH)
    % Draw a sample arm configuration in 2D (XZ projection)
    T = eye(4);
    pts = zeros(size(DH,1)+1, 2);
    pts(1,:) = [0, 0];
    for i = 1:size(DH,1)
        a_i  = DH(i,1); d_i  = DH(i,2);
        al_i = DH(i,3); th_i = theta(i) + DH(i,4);
        ct=cos(th_i);st=sin(th_i);ca=cos(al_i);sa=sin(al_i);
        T_i=[ct,-st*ca,st*sa,a_i*ct;st,ct*ca,-ct*sa,a_i*st;0,sa,ca,d_i;0,0,0,1];
        T   = T * T_i;
        pts(i+1,:) = [T(1,4), T(3,4)];   % X and Z
    end
    plot(ax, pts(:,1), pts(:,2), 'm-o','LineWidth',2.5,'MarkerSize',7,...
        'MarkerFaceColor','m','DisplayName','Sample config');
end
