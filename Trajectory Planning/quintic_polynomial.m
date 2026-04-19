% =========================================================================
% QUINTIC POLYNOMIAL TRAJECTORY — Cubic vs Quintic Comparison
% =========================================================================
% Polynomial trajectories interpolate smoothly between two configurations.
%
% CUBIC POLYNOMIAL (degree 3):  a₀ + a₁t + a₂t² + a₃t³
%   Constraints: q(0), q(T), q̇(0), q̇(T)  — 4 equations, 4 unknowns
%   ✓ Position and velocity are continuous
%   ✖ Acceleration has DISCONTINUITIES at start and end
%   ✖ Jerk (dq̈/dt) is infinite at boundaries — bad for motors & wear
%
% QUINTIC POLYNOMIAL (degree 5):  a₀ + a₁t + ... + a₅t⁵
%   Constraints: q(0), q(T), q̇(0), q̇(T), q̈(0), q̈(T)  — 6 equations
%   ✓ Position, velocity AND acceleration are continuous
%   ✓ Smoother motor commands, less mechanical wear
%   ✓ Required for high-speed or precision robotics
%
% WHAT THIS SCRIPT SHOWS:
%   1. Position, velocity, acceleration profiles for BOTH polynomials
%   2. Why the cubic acceleration profile is problematic (step change at t=0)
%   3. Phase-plane plots (q vs q̇) showing the trajectory shape
%   4. Animated 2-DOF arm following both trajectories simultaneously
%
% Boundary conditions are set to zero velocity and acceleration at
% start and end (most common case — the "rest to rest" motion).
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Quintic Polynomial — Cubic vs Quintic Comparison\n');
fprintf('============================================================\n\n');

% --- Motion parameters ---
T  = 2.0;           % motion duration (s)
dt = 0.005;
t  = 0:dt:T;
N  = length(t);

fprintf('Motion duration: T = %.1f s\n', T);
fprintf('Boundary conditions (same for both):\n');
fprintf('  q(0)=0°  q(T)=90°  q̇(0)=0  q̇(T)=0  q̈(0)=0  q̈(T)=0\n\n');

% --- Joint boundary conditions ---
q0   = 0;   qf   = deg2rad(90);   % position (rad)
qd0  = 0;   qdf  = 0;             % velocity (rad/s)
qdd0 = 0;   qddf = 0;             % acceleration (rad/s²)

% =========================================================================
%  CUBIC POLYNOMIAL COEFFICIENTS
%  System: [1 0  0   0 ] [a0]   [q0 ]
%          [1 T  T²  T³] [a1] = [qf ]
%          [0 1  0   0 ] [a2]   [qd0]
%          [0 1 2T  3T²] [a3]   [qdf]
% =========================================================================
A_cub = [1,  0,    0,     0;
         1,  T,    T^2,   T^3;
         0,  1,    0,     0;
         0,  1,   2*T,   3*T^2];
b_cub = [q0; qf; qd0; qdf];
a_cub = A_cub \ b_cub;

q_cub   = polyval_traj(a_cub, t, 0);
qd_cub  = polyval_traj(a_cub, t, 1);
qdd_cub = polyval_traj(a_cub, t, 2);

% =========================================================================
%  QUINTIC POLYNOMIAL COEFFICIENTS
%  System: 6×6 Vandermonde-like matrix
% =========================================================================
A_qui = [1,  0,    0,      0,      0,      0;
         1,  T,    T^2,    T^3,    T^4,    T^5;
         0,  1,    0,      0,      0,      0;
         0,  1,   2*T,    3*T^2,  4*T^3,  5*T^4;
         0,  0,    2,      0,      0,      0;
         0,  0,    2,     6*T,   12*T^2, 20*T^3];
b_qui = [q0; qf; qd0; qdf; qdd0; qddf];
a_qui = A_qui \ b_qui;

q_qui   = polyval_traj(a_qui, t, 0);
qd_qui  = polyval_traj(a_qui, t, 1);
qdd_qui = polyval_traj(a_qui, t, 2);
qddd_qui = polyval_traj(a_qui, t, 3);   % jerk
qddd_cub = polyval_traj(a_cub, t, 3);

fprintf('Cubic polynomial coefficients:\n');
fprintf('  a = [%.4f, %.4f, %.4f, %.4f]\n', a_cub);
fprintf('\nQuintic polynomial coefficients:\n');
fprintf('  a = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', a_qui);

% Boundary check
fprintf('\nBoundary condition verification:\n');
fprintf('           Cubic    Quintic   Target\n');
fprintf('  q(0):   %6.3f   %6.3f   %6.3f rad\n', q_cub(1),   q_qui(1),   q0);
fprintf('  q(T):   %6.3f   %6.3f   %6.3f rad\n', q_cub(end), q_qui(end), qf);
fprintf('  q̇(0):   %6.3f   %6.3f   %6.3f rad/s\n', qd_cub(1),  qd_qui(1),  qd0);
fprintf('  q̇(T):   %6.3f   %6.3f   %6.3f rad/s\n', qd_cub(end),qd_qui(end),qdf);
fprintf('  q̈(0):   %6.3f   %6.3f   %6.3f rad/s² (cubic cannot constrain)\n',...
    qdd_cub(1), qdd_qui(1), qdd0);
fprintf('  q̈(T):   %6.3f   %6.3f   %6.3f rad/s²\n', qdd_cub(end),qdd_qui(end),qddf);

% =========================================================================
%  FIGURE 1 — Profile Comparison (4 rows × 2 cols)
% =========================================================================
c_cub = [0.15 0.45 0.85];   % blue  — cubic
c_qui = [0.85 0.35 0.10];   % red   — quintic

fig1 = figure('Name','Cubic vs Quintic — Trajectory Profiles',...
    'Color','w','Position',[40 40 1300 700]);

plot_titles  = {'Position  q(t)  [deg]',...
                'Velocity  q̇(t)  [deg/s]',...
                'Acceleration  q̈(t)  [deg/s²]',...
                'Jerk  q⃛(t)  [deg/s³]'};
cubic_data   = {rad2deg(q_cub), rad2deg(qd_cub), rad2deg(qdd_cub), rad2deg(qddd_cub)};
quintic_data = {rad2deg(q_qui), rad2deg(qd_qui), rad2deg(qdd_qui), rad2deg(qddd_qui)};

for row = 1:4
    ax_l = subplot(4,2,(row-1)*2+1);
    hold(ax_l,'on'); grid(ax_l,'on');
    plot(ax_l, t, cubic_data{row},  '-','Color',c_cub,'LineWidth',2.5,...
        'DisplayName','Cubic');
    plot(ax_l, t, quintic_data{row},'-','Color',c_qui,'LineWidth',2.5,...
        'DisplayName','Quintic');
    yline(ax_l,0,'k-','LineWidth',0.8);
    ylabel(ax_l, plot_titles{row},'FontSize',9);
    if row==4, xlabel(ax_l,'Time (s)'); end
    if row==1, legend(ax_l,'Location','best','FontSize',10); end
    grid(ax_l,'on');

    % Zoom panel for acceleration discontinuity
    ax_r = subplot(4,2,(row-1)*2+2);
    hold(ax_r,'on'); grid(ax_r,'on');

    if row == 3
        % Zoom into start region to show cubic discontinuity
        zoom_end = 0.15;
        mask = t <= zoom_end;
        plot(ax_r, t(mask), cubic_data{row}(mask),  'o-','Color',c_cub,...
            'LineWidth',2.5,'MarkerSize',4,'DisplayName','Cubic');
        plot(ax_r, t(mask), quintic_data{row}(mask), 's-','Color',c_qui,...
            'LineWidth',2.5,'MarkerSize',4,'DisplayName','Quintic');
        yline(ax_r,0,'k-','LineWidth',0.8);
        title(ax_r,'↑ ZOOM: Acc at t≈0  |  Cubic jumps, Quintic=0',...
            'FontSize',9,'FontWeight','bold','Color',[0.7 0 0]);
        xlabel(ax_r,'Time (s)');
        legend(ax_r,'Location','best','FontSize',9);

        % Annotate the discontinuity
        [~,idx0] = min(abs(t-0.001));
        text(ax_r, t(idx0)+0.005, cubic_data{row}(idx0),...
            sprintf('  Cubic q̈(0)=%.2f°/s²\n  ← discontinuity!',...
            cubic_data{3}(1)),...
            'Color',c_cub,'FontSize',8,'FontWeight','bold');
        text(ax_r, t(idx0)+0.005, 0.5,...
            '  Quintic q̈(0)=0 ✓','Color',c_qui,'FontSize',8,'FontWeight','bold');

    elseif row == 4
        % Show jerk comparison — cubic has infinite jerk at endpoints
        plot(ax_r, t, cubic_data{row},  '-','Color',c_cub,'LineWidth',2);
        plot(ax_r, t, quintic_data{row},'-','Color',c_qui,'LineWidth',2);
        title(ax_r,'Jerk: cubic = constant (finite), quintic = smooth',...
            'FontSize',9,'FontWeight','bold');
        xlabel(ax_r,'Time (s)');
        yline(ax_r,0,'k-','LineWidth',0.8);

    else
        % Phase portrait: q vs q̇
        plot(ax_r, rad2deg(q_cub), rad2deg(qd_cub), '-','Color',c_cub,...
            'LineWidth',2.5,'DisplayName','Cubic');
        plot(ax_r, rad2deg(q_qui), rad2deg(qd_qui), '-','Color',c_qui,...
            'LineWidth',2.5,'DisplayName','Quintic');
        scatter(ax_r,[0 rad2deg(qf)],[0 0],80,'k','filled');
        xlabel(ax_r,'q (deg)'); ylabel(ax_r,'q̇ (deg/s)');
        title(ax_r,'Phase Portrait  (q vs q̇)','FontSize',9,'FontWeight','bold');
        legend(ax_r,'Location','best','FontSize',9);
        if row==1, title(ax_r,'Phase Portrait — both start/end at rest','FontSize',9); end
    end
end

sgtitle('Cubic vs Quintic Polynomial Trajectory  |  Rest-to-Rest Motion',...
    'FontSize',13,'FontWeight','bold');

% =========================================================================
%  FIGURE 2 — Animated 2-DOF arm following both trajectories
% =========================================================================
fprintf('\nOpening animation figure...\n');

L_arm = [1.0, 0.8];
% Use trajectory for joint 2 only; joint 1 stays at 30°
q1_fixed = deg2rad(30);

fig2 = figure('Name','Cubic vs Quintic — Arm Animation',...
    'Color','w','Position',[80 80 1000 500]);

ax_cub_arm = subplot(1,2,1);
hold(ax_cub_arm,'on'); grid(ax_cub_arm,'on'); axis(ax_cub_arm,'equal');
lim = sum(L_arm)+0.2;
axis(ax_cub_arm,[-0.3 lim+0.2 -0.3 lim+0.3]);
xlabel(ax_cub_arm,'X (m)'); ylabel(ax_cub_arm,'Y (m)');

ax_qui_arm = subplot(1,2,2);
hold(ax_qui_arm,'on'); grid(ax_qui_arm,'on'); axis(ax_qui_arm,'equal');
axis(ax_qui_arm,[-0.3 lim+0.2 -0.3 lim+0.3]);
xlabel(ax_qui_arm,'X (m)'); ylabel(ax_qui_arm,'Y (m)');

ee_cub_hist = zeros(N,2);
ee_qui_hist = zeros(N,2);

for k = 1:2:N   % step by 2 for speed
    q_c = [q1_fixed, q_cub(k)];
    q_q = [q1_fixed, q_qui(k)];
    [joints_c, ee_c] = fk_planar_2(q_c, L_arm);
    [joints_q, ee_q] = fk_planar_2(q_q, L_arm);
    ee_cub_hist(k,:) = ee_c;
    ee_qui_hist(k,:) = ee_q;

    % Cubic arm
    cla(ax_cub_arm); hold(ax_cub_arm,'on'); grid(ax_cub_arm,'on'); axis(ax_cub_arm,'equal');
    axis(ax_cub_arm,[-0.3 lim+0.2 -0.3 lim+0.3]);
    plot(ax_cub_arm, ee_cub_hist(1:k,1),ee_cub_hist(1:k,2),...
        '-','Color',c_cub,'LineWidth',1.5);
    draw_arm_2d(ax_cub_arm, joints_c, c_cub);
    title(ax_cub_arm,...
        sprintf('CUBIC\nq₂=%.1f°  q̈₂=%.2f°/s²',rad2deg(q_cub(k)),rad2deg(qdd_cub(k))),...
        'FontSize',11,'FontWeight','bold','Color',c_cub);
    xlabel(ax_cub_arm,'X (m)'); ylabel(ax_cub_arm,'Y (m)');

    % Quintic arm
    cla(ax_qui_arm); hold(ax_qui_arm,'on'); grid(ax_qui_arm,'on'); axis(ax_qui_arm,'equal');
    axis(ax_qui_arm,[-0.3 lim+0.2 -0.3 lim+0.3]);
    plot(ax_qui_arm, ee_qui_hist(1:k,1),ee_qui_hist(1:k,2),...
        '-','Color',c_qui,'LineWidth',1.5);
    draw_arm_2d(ax_qui_arm, joints_q, c_qui);
    title(ax_qui_arm,...
        sprintf('QUINTIC\nq₂=%.1f°  q̈₂=%.2f°/s²',rad2deg(q_qui(k)),rad2deg(qdd_qui(k))),...
        'FontSize',11,'FontWeight','bold','Color',c_qui);
    xlabel(ax_qui_arm,'X (m)'); ylabel(ax_qui_arm,'Y (m)');

    drawnow; pause(0.01);
end

sgtitle('Arm Motion: Cubic (sudden acc) vs Quintic (smooth acc)',...
    'FontSize',12,'FontWeight','bold');

fprintf('Done. Key takeaway:\n');
fprintf('  Cubic:   q̈(0) = %.4f rad/s² — JUMPS from 0 at t=0⁻ to this at t=0⁺\n',...
    qdd_cub(1));
fprintf('  Quintic: q̈(0) = %.2e rad/s² — starts smoothly from 0\n', qdd_qui(1));

% =========================================================================
function y = polyval_traj(a, t, deriv)
    % Evaluate polynomial or its derivative at times t
    n  = length(a)-1;   % degree
    y  = zeros(size(t));
    for k = 0:n
        coeff = a(k+1);
        if deriv == 0
            y = y + coeff * t.^k;
        elseif k >= deriv
            % d^d/dt^d (t^k) = k!/(k-d)! * t^(k-d)
            c = prod(k:-1:(k-deriv+1));
            y = y + coeff * c * t.^(k-deriv);
        end
    end
end

function [joints, ee] = fk_planar_2(q, L)
    joints = zeros(3,2);
    joints(2,:) = L(1)*[cos(q(1)),sin(q(1))];
    joints(3,:) = joints(2,:)+L(2)*[cos(q(1)+q(2)),sin(q(1)+q(2))];
    ee = joints(3,:);
end

function draw_arm_2d(ax, joints, col)
    for k=1:size(joints,1)-1
        plot(ax,[joints(k,1),joints(k+1,1)],[joints(k,2),joints(k+1,2)],...
            '-','Color',col,'LineWidth',5);
    end
    for k=1:size(joints,1)
        scatter(ax,joints(k,1),joints(k,2),55,...
            'MarkerFaceColor',[0.15 0.15 0.15],'MarkerEdgeColor','k');
    end
    scatter(ax,0,0,80,'k','filled','Marker','square');
end
