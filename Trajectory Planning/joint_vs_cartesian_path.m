% =========================================================================
% JOINT SPACE vs CARTESIAN SPACE PATH — Side-by-Side Comparison
% =========================================================================
% One of the most important and surprising results in robotics:
%
%   Moving joints linearly (joint space) does NOT produce a
%   straight-line end-effector path in Cartesian space — and vice versa.
%
% JOINT SPACE interpolation:
%   θ(t) = θ_start + s(t)·(θ_end - θ_start)
%   Simple and fast. Joints move uniformly.
%   EE path is a CURVE — can arc unexpectedly, may hit obstacles.
%
% CARTESIAN SPACE interpolation:
%   x(t) = x_start + s(t)·(x_end - x_start)
%   EE moves in a straight line — predictable, safe near obstacles.
%   Requires solving IK at every timestep — computationally heavier.
%   Can pass through singularities mid-path!
%
% BOTH animations run simultaneously so students can directly compare:
%   - The EE path shape
%   - How joints move differently to achieve each
%   - Where the paths diverge most
%
% Robot: 3-DOF planar arm  L = [1.0, 0.8, 0.5] m
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Joint Space vs Cartesian Space Path Comparison\n');
fprintf('============================================================\n\n');

% --- Robot ---
L = [1.0, 0.8, 0.5];
n = length(L);

% --- Start and end configurations ---
% Choose configs that produce a visually dramatic difference
q_start = deg2rad([20,  -10,  30]);
q_end   = deg2rad([80, -110,  60]);

% Compute start/end EE positions via FK
[~, ee_start] = fk_planar(q_start, L);
[~, ee_end  ] = fk_planar(q_end,   L);

fprintf('Start config:  q = [%.1f°, %.1f°, %.1f°]\n', rad2deg(q_start));
fprintf('End   config:  q = [%.1f°, %.1f°, %.1f°]\n', rad2deg(q_end));
fprintf('Start EE pos:  (%.3f, %.3f) m\n', ee_start(1), ee_start(2));
fprintf('End   EE pos:  (%.3f, %.3f) m\n', ee_end(1),   ee_end(2));
fprintf('\nCartesian straight-line distance: %.3f m\n', norm(ee_end-ee_start));

% --- Trajectory parameters ---
T      = 3.0;
dt     = 0.03;
t_vec  = 0:dt:T;
N      = length(t_vec);

% Smooth trapezoidal s(t) profile — same for both methods
s_vec  = smooth_profile(t_vec, T);

% -------------------------------------------------------------------------
%  JOINT SPACE PATH: interpolate θ linearly
% -------------------------------------------------------------------------
q_js  = zeros(N, n);
ee_js = zeros(N, 2);
for k = 1:N
    q_js(k,:)  = q_start + s_vec(k)*(q_end - q_start);
    [~, ee_js(k,:)] = fk_planar(q_js(k,:), L);
end

% -------------------------------------------------------------------------
%  CARTESIAN SPACE PATH: interpolate EE linearly, solve IK at each step
% -------------------------------------------------------------------------
q_cs  = zeros(N, n);
ee_cs = zeros(N, 2);
q_cs(1,:) = q_start;

for k = 1:N
    % Desired EE position (straight line)
    ee_d = ee_start + s_vec(k)*(ee_end - ee_start);
    ee_cs(k,:) = ee_d;

    % Solve IK from previous joint angles (warm start)
    q_prev = q_cs(max(k-1,1),:);
    q_cs(k,:) = ik_numerical(ee_d, q_prev, L);
end

% -------------------------------------------------------------------------
%  Compute how much the paths differ
% -------------------------------------------------------------------------
path_diff = sqrt(sum((ee_js - ee_cs).^2, 2));
[max_diff, max_idx] = max(path_diff);
fprintf('Max EE deviation between paths: %.4f m at t=%.2f s\n',...
    max_diff, t_vec(max_idx));

% -------------------------------------------------------------------------
%  Build figure — two arm plots + comparison + joint angle plots
% -------------------------------------------------------------------------
fig = figure('Name','Joint Space vs Cartesian Space Path',...
    'Color','w','Position',[30 30 1400 720]);

% Left: Joint space arm
ax_js = subplot(2,3,1);
hold(ax_js,'on'); grid(ax_js,'on'); axis(ax_js,'equal');
lim = sum(L)+0.3;
axis(ax_js,[-lim lim -lim lim]);
xlabel(ax_js,'X (m)'); ylabel(ax_js,'Y (m)');

% Right: Cartesian space arm
ax_cs = subplot(2,3,2);
hold(ax_cs,'on'); grid(ax_cs,'on'); axis(ax_cs,'equal');
axis(ax_cs,[-lim lim -lim lim]);
xlabel(ax_cs,'X (m)'); ylabel(ax_cs,'Y (m)');

% Centre-right: EE path overlay
ax_path = subplot(2,3,3);
hold(ax_path,'on'); grid(ax_path,'on'); axis(ax_path,'equal');
xlabel(ax_path,'X (m)'); ylabel(ax_path,'Y (m)');
title(ax_path,'EE Path Overlay','FontSize',11,'FontWeight','bold');

% Bottom: joint angle histories
ax_q1 = subplot(2,3,4);
hold(ax_q1,'on'); grid(ax_q1,'on');
xlabel(ax_q1,'t (s)'); ylabel(ax_q1,'Angle (°)');
title(ax_q1,'Joint Angles — Joint Space','FontSize',10,'FontWeight','bold');

ax_q2 = subplot(2,3,5);
hold(ax_q2,'on'); grid(ax_q2,'on');
xlabel(ax_q2,'t (s)'); ylabel(ax_q2,'Angle (°)');
title(ax_q2,'Joint Angles — Cartesian Space','FontSize',10,'FontWeight','bold');

% Bottom-right: path deviation
ax_dev = subplot(2,3,6);
hold(ax_dev,'on'); grid(ax_dev,'on');
xlabel(ax_dev,'t (s)'); ylabel(ax_dev,'Deviation (m)');
title(ax_dev,'EE Path Deviation |p_{JS} - p_{CS}|','FontSize',10,'FontWeight','bold');

% Static elements on path overlay
plot(ax_path, ee_js(:,1),  ee_js(:,2),  '-', 'Color',[0.15 0.45 0.85],...
    'LineWidth',2.5,'DisplayName','Joint Space path');
plot(ax_path, ee_cs(:,1),  ee_cs(:,2),  '-', 'Color',[0.85 0.35 0.10],...
    'LineWidth',2.5,'DisplayName','Cartesian Space path');
plot(ax_path, [ee_start(1),ee_end(1)], [ee_start(2),ee_end(2)],...
    'k--','LineWidth',1.5,'DisplayName','True straight line');
scatter(ax_path,[ee_start(1),ee_end(1)],[ee_start(2),ee_end(2)],...
    80,'k','filled');
text(ax_path,ee_start(1)+0.05,ee_start(2)+0.08,'Start','FontSize',9);
text(ax_path,ee_end(1)+0.05,  ee_end(2)+0.08,  'End',  'FontSize',9);
legend(ax_path,'Location','best','FontSize',9);

% Full deviation curve
plot(ax_dev, t_vec, path_diff, 'k-','LineWidth',2);
xline(ax_dev, t_vec(max_idx),'r--','LineWidth',1.5,...
    'Label',sprintf('Max: %.3fm',max_diff),'FontSize',9,...
    'LabelVerticalAlignment','bottom');
fill(ax_dev,[t_vec,fliplr(t_vec)],[path_diff',zeros(1,N)],...
    [0.9 0.7 0.7],'FaceAlpha',0.3,'EdgeColor','none');

% Full joint angle curves (static background)
jcols = {[0.2 0.5 0.9],[0.9 0.4 0.1],[0.1 0.7 0.3]};
for j=1:n
    plot(ax_q1, t_vec, rad2deg(q_js(:,j)),'-','Color',jcols{j},...
        'LineWidth',2,'DisplayName',sprintf('θ%d',j));
    plot(ax_q2, t_vec, rad2deg(q_cs(:,j)),'-','Color',jcols{j},...
        'LineWidth',2,'DisplayName',sprintf('θ%d',j));
end
legend(ax_q1,'Location','best','FontSize',9);
legend(ax_q2,'Location','best','FontSize',9);

% -------------------------------------------------------------------------
%  Animate both simultaneously
% -------------------------------------------------------------------------
c_js = [0.15 0.45 0.85];   % blue  — joint space
c_cs = [0.85 0.35 0.10];   % red   — Cartesian space

h_arm_js=[]; h_arm_cs=[];
h_trail_js=[]; h_trail_cs=[];
h_tline1=[]; h_tline2=[];

for k = 1:N
    delete([h_arm_js; h_arm_cs; h_trail_js; h_trail_cs; h_tline1; h_tline2]);

    % Draw Joint Space arm
    [joints_js,~] = fk_planar(q_js(k,:), L);
    cla(ax_js); hold(ax_js,'on'); grid(ax_js,'on'); axis(ax_js,'equal');
    axis(ax_js,[-lim lim -lim lim]);
    h_arm_js  = draw_arm(ax_js, joints_js, c_js);
    h_trail_js = plot(ax_js, ee_js(1:k,1), ee_js(1:k,2),...
        '-','Color',c_js,'LineWidth',2);
    scatter(ax_js,ee_end(1),ee_end(2),80,'k','filled','Marker','pentagram');
    title(ax_js,sprintf('JOINT SPACE\nt = %.2f s  |  s = %.2f',t_vec(k),s_vec(k)),...
        'FontSize',11,'FontWeight','bold','Color',c_js);
    xlabel(ax_js,'X (m)'); ylabel(ax_js,'Y (m)');

    % Draw Cartesian Space arm
    [joints_cs,~] = fk_planar(q_cs(k,:), L);
    cla(ax_cs); hold(ax_cs,'on'); grid(ax_cs,'on'); axis(ax_cs,'equal');
    axis(ax_cs,[-lim lim -lim lim]);
    h_arm_cs  = draw_arm(ax_cs, joints_cs, c_cs);
    h_trail_cs = plot(ax_cs, ee_cs(1:k,1), ee_cs(1:k,2),...
        '-','Color',c_cs,'LineWidth',2);
    % Show desired straight line
    plot(ax_cs,[ee_start(1),ee_end(1)],[ee_start(2),ee_end(2)],...
        'k--','LineWidth',1.5);
    scatter(ax_cs,ee_end(1),ee_end(2),80,'k','filled','Marker','pentagram');
    title(ax_cs,sprintf('CARTESIAN SPACE\nt = %.2f s  |  EE on straight line',t_vec(k)),...
        'FontSize',11,'FontWeight','bold','Color',c_cs);
    xlabel(ax_cs,'X (m)'); ylabel(ax_cs,'Y (m)');

    % Time markers on joint angle plots
    h_tline1 = xline(ax_q1, t_vec(k),'k-','LineWidth',1.5);
    h_tline2 = xline(ax_q2, t_vec(k),'k-','LineWidth',1.5);

    drawnow;
    pause(0.02);
end

sgtitle('Joint Space vs Cartesian Space  |  Same Start & End, Very Different Paths',...
    'FontSize',13,'FontWeight','bold');

fprintf('\nKey insight:\n');
fprintf('  Joint space interpolation: simple, but EE takes a CURVED path.\n');
fprintf('  Cartesian interpolation:   EE moves straight, but requires IK every step.\n');
fprintf('  Maximum path deviation observed: %.4f m\n', max_diff);

% =========================================================================
function s = smooth_profile(t, T)
    % Smooth s(t) using cosine — zero velocity at start and end
    s = (1 - cos(pi*t/T)) / 2;
end

function [joints, ee] = fk_planar(q, L)
    n = length(L);
    joints = zeros(n+1,2);
    cum = 0;
    for i=1:n
        cum = cum+q(i);
        joints(i+1,:) = joints(i,:)+L(i)*[cos(cum),sin(cum)];
    end
    ee = joints(end,:);
end

function q_sol = ik_numerical(ee_d, q0, L)
    q      = q0;
    gain   = 0.5;
    lambda = 0.05;
    for iter = 1:80
        [~,ee] = fk_planar(q,L);
        err    = ee_d(:) - ee(:);
        if norm(err)<1e-4, break; end
        J      = planar_jacobian(q,L);
        dq     = gain * J'/(J*J'+lambda^2*eye(2)) * err;
        q      = q + dq';
    end
    q_sol = q;
end

function J = planar_jacobian(q, L)
    n = length(L);
    J = zeros(2,n);
    [joints,ee] = fk_planar(q,L);
    for i=1:n
        r = ee-joints(i,:);
        J(:,i) = [-r(2);r(1)];
    end
end

function h = draw_arm(ax, joints, col)
    h = [];
    for k=1:size(joints,1)-1
        h(end+1) = plot(ax,[joints(k,1),joints(k+1,1)],...
            [joints(k,2),joints(k+1,2)],'-','Color',col,...
            'LineWidth',5); %#ok<AGROW>
    end
    for k=1:size(joints,1)
        h(end+1) = scatter(ax,joints(k,1),joints(k,2),55,... %#ok<AGROW>
            'MarkerFaceColor',[0.15 0.15 0.15],'MarkerEdgeColor','k');
    end
    h(end+1) = scatter(ax,0,0,80,'k','filled','Marker','square'); %#ok<AGROW>
end
