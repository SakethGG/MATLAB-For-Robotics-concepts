% =========================================================================
% M, C, G MATRIX VISUALISATION — How Dynamics Change With Configuration
% =========================================================================
% The robot equation of motion is:
%
%   M(q)·q̈  +  C(q,q̇)·q̇  +  g(q)  =  τ
%    ↑               ↑            ↑
%  Inertia      Coriolis/     Gravity
%  matrix      Centrifugal    vector
%
% This script builds physical intuition for each term:
%
%   M(q)   — Inertia matrix: how hard it is to accelerate the arm.
%             Changes with configuration because link mass distribution changes.
%             Always symmetric and positive definite.
%
%   C(q,q̇) — Coriolis/centrifugal matrix: coupling forces between joints.
%             Zero when the arm is stationary (q̇ = 0).
%             Grows quadratically with joint velocity.
%
%   g(q)   — Gravity vector: torque each joint must overcome just to hold pose.
%             Largest when links are horizontal, zero when vertical.
%
% TWO VIEWS:
%   PART 1 — Static sweep: vary θ2 from -180° to 180°, plot each term
%   PART 2 — Live animation: arm moves along a trajectory, bars update live
%
% Robot: 2-DOF planar arm (analytical M/C/g available in closed form)
%   m = [1.0, 0.8] kg,  L = [1.0, 0.8] m,  lc = L/2
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  M, C, G Matrix Visualisation  |  2-DOF Planar Arm\n');
fprintf('============================================================\n\n');

% --- Robot parameters ---
m  = [1.0, 0.8];      % link masses (kg)
L  = [1.0, 0.8];      % link lengths (m)
lc = L / 2;           % CoM at midpoint
g_acc = 9.81;         % gravity (m/s²)

fprintf('Robot parameters:\n');
fprintf('  m = [%.1f, %.1f] kg\n', m(1), m(2));
fprintf('  L = [%.1f, %.1f] m\n',  L(1), L(2));
fprintf('  lc= [%.2f, %.2f] m  (CoM at link midpoint)\n\n', lc(1), lc(2));

% =========================================================================
%  PART 1 — STATIC SWEEP: vary θ2, fix θ1 = 30°
% =========================================================================
fprintf('PART 1: Static parameter sweep...\n');

theta1_fixed = deg2rad(30);
theta2_sweep = linspace(-pi, pi, 360);
qd_fixed     = [1.5; 1.0];    % fixed joint velocity for C term

% Pre-allocate
M11=zeros(1,360); M12=zeros(1,360); M22=zeros(1,360);
C1 =zeros(1,360); C2 =zeros(1,360);
g1 =zeros(1,360); g2 =zeros(1,360);

for k = 1:360
    q  = [theta1_fixed; theta2_sweep(k)];
    Mk = M_matrix(q, m, L, lc);
    Ck = C_matrix(q, qd_fixed, m, L, lc);
    gk = g_vector(q, m, L, lc, g_acc);

    M11(k) = Mk(1,1);  M12(k) = Mk(1,2);  M22(k) = Mk(2,2);
    Cqd    = Ck * qd_fixed;
    C1(k)  = Cqd(1);   C2(k)  = Cqd(2);
    g1(k)  = gk(1);    g2(k)  = gk(2);
end

theta2_deg = rad2deg(theta2_sweep);

% --- Figure 1: Static sweep ---
fig1 = figure('Name','M/C/G Static Sweep','Color','w',...
    'Position',[40 60 1350 700]);

% Colour scheme
c_M = [0.15 0.45 0.85];
c_C = [0.85 0.35 0.10];
c_g = [0.10 0.65 0.25];

% --- M(q) subplot ---
ax_M = subplot(2,3,1);
hold(ax_M,'on'); grid(ax_M,'on');
plot(ax_M, theta2_deg, M11, '-',  'Color',c_M, 'LineWidth',2.5, 'DisplayName','M_{11}');
plot(ax_M, theta2_deg, M12, '--', 'Color',c_M, 'LineWidth',2.0, 'DisplayName','M_{12}=M_{21}');
plot(ax_M, theta2_deg, M22, ':',  'Color',c_M, 'LineWidth',2.0, 'DisplayName','M_{22}');
xlabel(ax_M,'θ₂ (deg)'); ylabel(ax_M,'Inertia (kg·m²)');
title(ax_M,'M(q) — Inertia Matrix','FontSize',11,'FontWeight','bold','Color',c_M);
legend(ax_M,'Location','best','FontSize',9);
xline(ax_M, 0,'k--','LineWidth',1);

% Annotation: physical meaning
annotation(fig1,'textbox',[0.02 0.89 0.30 0.08],...
    'String',['M(q): Resistance to acceleration. ',...
    'M_{11} is largest — joint 1 must accelerate entire arm. ',...
    'M_{12} = coupling inertia — accelerating joint 2 creates reaction at joint 1.'],...
    'FontSize',8,'EdgeColor',c_M,'BackgroundColor',[0.93 0.95 1.0],'Interpreter','none');

% --- C(q,q̇) subplot ---
ax_C = subplot(2,3,2);
hold(ax_C,'on'); grid(ax_C,'on');
plot(ax_C, theta2_deg, C1, '-',  'Color',c_C, 'LineWidth',2.5, 'DisplayName','(C·q̇)₁');
plot(ax_C, theta2_deg, C2, '--', 'Color',c_C, 'LineWidth',2.0, 'DisplayName','(C·q̇)₂');
xlabel(ax_C,'θ₂ (deg)'); ylabel(ax_C,'Force (N·m)');
title(ax_C,sprintf('C(q,q̇)·q̇ — Coriolis/Centrifugal\n(q̇=[%.1f, %.1f] rad/s)',...
    qd_fixed(1),qd_fixed(2)),'FontSize',11,'FontWeight','bold','Color',c_C);
legend(ax_C,'Location','best','FontSize',9);
xline(ax_C, 0,'k--','LineWidth',1);
yline(ax_C, 0,'k-','LineWidth',0.8);

annotation(fig1,'textbox',[0.355 0.89 0.30 0.08],...
    'String',['C(q,q̇)·q̇: Velocity-dependent forces. ',...
    'Zero when arm is stationary. ',...
    'Grows with sin(θ₂) — maximum coupling at θ₂=±90°.'],...
    'FontSize',8,'EdgeColor',c_C,'BackgroundColor',[1.0 0.95 0.92],'Interpreter','none');

% --- g(q) subplot ---
ax_g = subplot(2,3,3);
hold(ax_g,'on'); grid(ax_g,'on');
plot(ax_g, theta2_deg, g1, '-',  'Color',c_g, 'LineWidth',2.5, 'DisplayName','g₁(q)');
plot(ax_g, theta2_deg, g2, '--', 'Color',c_g, 'LineWidth',2.0, 'DisplayName','g₂(q)');
xlabel(ax_g,'θ₂ (deg)'); ylabel(ax_g,'Torque (N·m)');
title(ax_g,'g(q) — Gravity Vector','FontSize',11,'FontWeight','bold','Color',c_g);
legend(ax_g,'Location','best','FontSize',9);
xline(ax_g, 0,'k--','LineWidth',1);
yline(ax_g, 0,'k-','LineWidth',0.8);

annotation(fig1,'textbox',[0.69 0.89 0.30 0.08],...
    'String',['g(q): Gravity torque to hold pose. ',...
    'Largest when links are horizontal (θ=0°). ',...
    'g₁ dominates — joint 1 supports both links.'],...
    'FontSize',8,'EdgeColor',c_g,'BackgroundColor',[0.92 1.0 0.94],'Interpreter','none');

% --- Relative magnitude comparison ---
ax_rel = subplot(2,3,[4 5 6]);
hold(ax_rel,'on'); grid(ax_rel,'on');

% Total magnitude of each term across the sweep
mag_M = sqrt(M11.^2 + 2*M12.^2 + M22.^2);
mag_C = sqrt(C1.^2 + C2.^2);
mag_g = sqrt(g1.^2 + g2.^2);

plot(ax_rel, theta2_deg, mag_M, '-',  'Color',c_M, 'LineWidth',2.5, 'DisplayName','||M(q)||_F');
plot(ax_rel, theta2_deg, mag_C, '-',  'Color',c_C, 'LineWidth',2.5, 'DisplayName','||C·q̇||');
plot(ax_rel, theta2_deg, mag_g, '-',  'Color',c_g, 'LineWidth',2.5, 'DisplayName','||g(q)||');
xlabel(ax_rel,'θ₂ (deg)','FontSize',11);
ylabel(ax_rel,'Magnitude','FontSize',11);
title(ax_rel,...
    'Relative Magnitudes  |  Which term dominates at each configuration?',...
    'FontSize',11,'FontWeight','bold');
legend(ax_rel,'Location','best','FontSize',10);
xline(ax_rel, 0,'k--','LineWidth',1);

sgtitle('M(q), C(q,q̇), g(q)  |  Static Sweep  |  θ₁=30° fixed, θ₂ varies',...
    'FontSize',13,'FontWeight','bold');

% =========================================================================
%  PART 2 — LIVE ANIMATION: arm follows a sinusoidal trajectory
% =========================================================================
fprintf('PART 2: Live animation...\n');
fprintf('(Close Part 1 figure or leave it open — Part 2 opens separately)\n\n');

T_anim = 4.0;
dt     = 0.04;
t_vec  = 0:dt:T_anim;
N      = length(t_vec);

% Sinusoidal joint trajectory
omega  = pi;
q_traj = [30 + 60*sin(omega*t_vec);
          -45 + 80*sin(2*omega*t_vec + pi/4)];
qd_traj= [60*omega*cos(omega*t_vec);
          160*omega*cos(2*omega*t_vec + pi/4)] * pi/180;   % rad/s

fig2 = figure('Name','M/C/G Live Animation','Color','w',...
    'Position',[80 50 1300 660]);

ax_arm2 = subplot(2,4,[1,5]);
hold(ax_arm2,'on'); grid(ax_arm2,'on'); axis(ax_arm2,'equal');
lim = sum(L) + 0.2;
axis(ax_arm2,[-lim lim -lim lim]);
xlabel(ax_arm2,'X (m)'); ylabel(ax_arm2,'Y (m)');
title(ax_arm2,'Arm Motion','FontSize',11,'FontWeight','bold');

ax_bar_M = subplot(2,4,2);
ax_bar_C = subplot(2,4,3);
ax_bar_g = subplot(2,4,4);
ax_hist  = subplot(2,4,[6 7 8]);
hold(ax_hist,'on'); grid(ax_hist,'on');
xlabel(ax_hist,'Time (s)'); ylabel(ax_hist,'Magnitude');
title(ax_hist,'Term Magnitudes Over Time','FontSize',11,'FontWeight','bold');

hist_t=[]; hist_M=[]; hist_C=[]; hist_g=[];

for k = 1:N
    q_k  = deg2rad(q_traj(:,k));
    qd_k = qd_traj(:,k);

    Mk   = M_matrix(q_k, m, L, lc);
    Ck   = C_matrix(q_k, qd_k, m, L, lc);
    gk   = g_vector(q_k, m, L, lc, g_acc);
    Cqd  = Ck * qd_k;

    mM   = norm(Mk,'fro');
    mC   = norm(Cqd);
    mg   = norm(gk);

    hist_t(end+1) = t_vec(k); %#ok<AGROW>
    hist_M(end+1) = mM;       %#ok<AGROW>
    hist_C(end+1) = mC;       %#ok<AGROW>
    hist_g(end+1) = mg;       %#ok<AGROW>

    % --- Arm ---
    [joints, ~] = fk_planar(q_k, L);
    cla(ax_arm2); hold(ax_arm2,'on'); grid(ax_arm2,'on'); axis(ax_arm2,'equal');
    axis(ax_arm2,[-lim lim -lim lim]);
    plot(ax_arm2, joints(:,1), joints(:,2), 'b-o',...
        'LineWidth',4,'MarkerSize',8,'MarkerFaceColor','b');
    scatter(ax_arm2, 0,0,80,'k','filled','Marker','square');
    title(ax_arm2, sprintf('t = %.2f s\nθ=[%.1f°, %.1f°]  q̇=[%.1f, %.1f] rad/s',...
        t_vec(k), q_traj(1,k), q_traj(2,k), qd_k(1), qd_k(2)),...
        'FontSize',10,'FontWeight','bold');
    xlabel(ax_arm2,'X (m)'); ylabel(ax_arm2,'Y (m)');

    % --- Bar charts ---
    draw_bar_panel(ax_bar_M, [Mk(1,1), Mk(1,2), Mk(2,2)], ...
        {'M_{11}','M_{12}','M_{22}'}, 'M(q)  [kg·m²]', c_M);
    draw_bar_panel(ax_bar_C, [Cqd(1), Cqd(2)], ...
        {'(Cq̇)_1','(Cq̇)_2'}, 'C·q̇  [N·m]', c_C);
    draw_bar_panel(ax_bar_g, [gk(1), gk(2)], ...
        {'g_1','g_2'}, 'g(q)  [N·m]', c_g);

    % --- History lines ---
    cla(ax_hist); hold(ax_hist,'on'); grid(ax_hist,'on');
    plot(ax_hist, hist_t, hist_M, '-', 'Color',c_M,'LineWidth',2,'DisplayName','||M||_F');
    plot(ax_hist, hist_t, hist_C, '-', 'Color',c_C,'LineWidth',2,'DisplayName','||C·q̇||');
    plot(ax_hist, hist_t, hist_g, '-', 'Color',c_g,'LineWidth',2,'DisplayName','||g||');
    xline(ax_hist, t_vec(k),'k--','LineWidth',1.2);
    legend(ax_hist,'Location','northeast','FontSize',9);
    xlabel(ax_hist,'Time (s)'); ylabel(ax_hist,'Magnitude');
    title(ax_hist,'Term Magnitudes Over Time','FontSize',11,'FontWeight','bold');
    xlim(ax_hist,[0 T_anim]);

    drawnow;
    pause(0.02);
end

sgtitle(fig2,'M(q), C(q,q̇), g(q)  |  Live Animation Along Trajectory',...
    'FontSize',13,'FontWeight','bold');

fprintf('Animation complete.\n');

% =========================================================================
%  DYNAMICS FUNCTIONS
% =========================================================================
function M = M_matrix(q, m, L, lc)
    I1  = m(1)*lc(1)^2;
    I2  = m(2)*lc(2)^2;
    h   = m(2)*L(1)*lc(2)*cos(q(2));
    M   = [I1+I2+m(2)*L(1)^2+2*h,  I2+h;
                              I2+h,    I2];
end

function C = C_matrix(q, qd, m, L, lc)
    h   = m(2)*L(1)*lc(2)*sin(q(2));
    C   = [-h*qd(2),  -h*(qd(1)+qd(2));
            h*qd(1),   0              ];
end

function gv = g_vector(q, m, L, lc, g)
    g1 = (m(1)*lc(1)+m(2)*L(1))*g*cos(q(1)) + m(2)*lc(2)*g*cos(q(1)+q(2));
    g2 =  m(2)*lc(2)*g*cos(q(1)+q(2));
    gv = [g1; g2];
end

function [joints, ee] = fk_planar(q, L)
    joints = zeros(3,2);
    joints(2,:) = joints(1,:) + L(1)*[cos(q(1)),       sin(q(1))];
    joints(3,:) = joints(2,:) + L(2)*[cos(q(1)+q(2)),  sin(q(1)+q(2))];
    ee = joints(3,:);
end

function draw_bar_panel(ax, vals, labels, ttl, col)
    cla(ax);
    b = bar(ax, vals, 'FaceColor',col,'EdgeColor','none','BarWidth',0.6);
    set(ax,'XTickLabel',labels,'FontSize',9);
    title(ax, ttl,'FontSize',10,'FontWeight','bold','Color',col);
    grid(ax,'on');
    yline(ax, 0,'k-','LineWidth',0.8);
    % Value labels on bars
    for i = 1:length(vals)
        text(ax, i, vals(i)+sign(vals(i))*0.05*max(abs(vals)+0.01),...
            sprintf('%.2f',vals(i)),'HorizontalAlignment','center',...
            'FontSize',8,'FontWeight','bold');
    end
    ylim(ax,'auto');
end
