% =========================================================================
% SINGULARITY DETECTION — Approaching & Passing Through Singularities
% =========================================================================
% A robot singularity occurs when the Jacobian loses rank:
%   det(J·Jᵀ) = 0  →  the arm loses ability to move in some direction
%
% At a singularity:
%   - The manipulability ellipse COLLAPSES to a line (or point)
%   - det(J) = 0  →  J is not invertible
%   - Small EE velocities require INFINITE joint velocities
%   - The robot is "stuck" — cannot exert force/move in lost direction
%
% THREE SINGULARITY TYPES demonstrated for a 3-DOF planar arm:
%
%   TYPE 1 — FULLY EXTENDED (elbow straight out)
%     θ2 + θ3 = 0 or 180°  →  arm in a line, lost one DoF
%
%   TYPE 2 — FULLY FOLDED (elbow doubled back)
%     All links aligned, EE near base
%
%   TYPE 3 — WRIST SINGULARITY (for 3-DOF: θ3 = 0°)
%     Links 2 and 3 aligned  →  Jacobian columns become parallel
%
% FEATURES:
%   ✓ Auto-animates path through each singularity
%   ✓ Live plots of det(J) and manipulability w
%   ✓ Manipulability ellipse shown at each step
%   ✓ Known singular configurations listed
%   ✓ Velocity amplification demonstrated
%
% Robot: 3-DOF planar arm  L = [1.0, 0.8, 0.5] m
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Singularity Detection  |  3-DOF Planar Arm\n');
fprintf('============================================================\n\n');

L = [1.0, 0.8, 0.5];
n = length(L);

% -------------------------------------------------------------------------
%  Print known singularities first
% -------------------------------------------------------------------------
fprintf('KNOWN SINGULAR CONFIGURATIONS:\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Type 1 — Fully Extended:\n');
fprintf('    θ1=any, θ2=0°, θ3=0°  →  all links in a line\n');
fprintf('    Arm cannot move radially (no radial DoF)\n\n');
fprintf('  Type 2 — Elbow Lock:\n');
fprintf('    θ2 + θ3 = 180°  →  links 2 & 3 cancel each other\n');
fprintf('    EE moves on a circle — lost 1 DoF\n\n');
fprintf('  Type 3 — Wrist Aligned:\n');
fprintf('    θ3 = 0°  →  links 2 & 3 fully aligned\n');
fprintf('    Columns 2 & 3 of J become parallel\n');
fprintf('─────────────────────────────────────────────────────\n\n');

% -------------------------------------------------------------------------
%  Build figure layout
% -------------------------------------------------------------------------
fig = figure('Name','Singularity Detection','Color','w',...
    'Position',[40 30 1350 700]);

% Arm + ellipse plot
ax_arm = subplot(2,3,[1,4]);
hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
lim = sum(L)+0.3;
axis(ax_arm,[-lim lim -lim lim]);
xlabel(ax_arm,'X (m)','FontSize',11); ylabel(ax_arm,'Y (m)','FontSize',11);
title(ax_arm,'Arm + Manipulability Ellipse','FontSize',11,'FontWeight','bold');

% det(J) plot
ax_det = subplot(2,3,2);
hold(ax_det,'on'); grid(ax_det,'on');
xlabel(ax_det,'Step'); ylabel(ax_det,'det(J_v \cdot J_v^T)');
title(ax_det,'Determinant of J_v·J_vᵀ','FontSize',11,'FontWeight','bold');

% Manipulability w plot
ax_w = subplot(2,3,5);
hold(ax_w,'on'); grid(ax_w,'on');
xlabel(ax_w,'Step'); ylabel(ax_w,'w = \surd det(J_v \cdot J_v^T)');
title(ax_w,'Manipulability  w','FontSize',11,'FontWeight','bold');

% Velocity amplification plot
ax_vel = subplot(2,3,3);
hold(ax_vel,'on'); grid(ax_vel,'on');
xlabel(ax_vel,'Step'); ylabel(ax_vel,'||\Delta\theta|| for unit EE motion');
title(ax_vel,'Joint Velocity Amplification','FontSize',11,'FontWeight','bold');

% Info panel
ax_info = subplot(2,3,6);
axis(ax_info,'off');
info_h = text(ax_info,0.05,0.97,'','Units','normalized',...
    'VerticalAlignment','top','FontName','Courier','FontSize',9,...
    'Interpreter','none');

% -------------------------------------------------------------------------
%  Define THREE animation sequences — each approaches a singularity
% -------------------------------------------------------------------------
sequences = {
    % Type 1: approach fully extended (θ2→0, θ3→0)
    struct('name',   'Type 1: Fully Extended',...
           'color',  [0.85 0.2 0.2],...
           'q_start', deg2rad([0,  60,  90]),...
           'q_end',   deg2rad([0,   0,   0]),...
           'desc',   {'All links align in a straight line.',...
                      'rank(J) drops from 2 to 1.',...
                      'Cannot move radially (outward/inward).'}),
    % Type 2: elbow lock (θ2 + θ3 → 180°)
    struct('name',   'Type 2: Elbow Lock (θ₂+θ₃→180°)',...
           'color',  [0.1 0.5 0.9],...
           'q_start', deg2rad([30,  90,  30]),...
           'q_end',   deg2rad([30, 120,  60]),...
           'desc',   {'Links 2 & 3 form opposing angles.',...
                      'Jacobian cols 2 & 3 become parallel.',...
                      'Lost ability to change EE height.'}),
    % Type 3: wrist alignment (θ3 → 0)
    struct('name',   'Type 3: Wrist Aligned (θ₃→0°)',...
           'color',  [0.1 0.6 0.1],...
           'q_start', deg2rad([45, -60, 80]),...
           'q_end',   deg2rad([45, -60,  0]),...
           'desc',   {'Link 3 aligns with link 2.',...
                      'Last two Jacobian cols become parallel.',...
                      'Wrist loses independent control.'})
};

n_steps = 80;
all_colors = {[0.85 0.2 0.2], [0.1 0.5 0.9], [0.1 0.6 0.1]};

% -------------------------------------------------------------------------
%  Run each sequence
% -------------------------------------------------------------------------
for seq_idx = 1:length(sequences)
    seq      = sequences{seq_idx};
    col      = seq.color;
    q_start  = seq.q_start;
    q_end    = seq.q_end;

    det_hist = zeros(1, n_steps);
    w_hist   = zeros(1, n_steps);
    vel_hist = zeros(1, n_steps);
    step_ids = zeros(1, n_steps);

    % Offset step index so all three sequences appear on same plot
    offset = (seq_idx-1) * n_steps;

    fprintf('▶ Animating: %s\n', seq.name);

    for k = 1:n_steps
        s      = (k-1)/(n_steps-1);
        % Smooth ease-in so we slow down near the singularity
        s_ease = s^0.6;
        q_k    = q_start + s_ease*(q_end - q_start);

        % FK and Jacobian
        [joints, ee] = fk_planar(q_k, L);
        J  = planar_jacobian(q_k, L);   % 2×3
        JJt = J*J';
        d  = det(JJt);
        w  = sqrt(max(d, 0));

        % Velocity amplification: ||dq|| for unit EE motion
        lambda = 0.01;
        J_dls  = J' / (JJt + lambda^2*eye(2));
        v_unit = [1; 0];   % unit motion in X
        dq_mag = norm(J_dls * v_unit);

        det_hist(k) = d;
        w_hist(k)   = w;
        vel_hist(k) = min(dq_mag, 30);   % cap for display
        step_ids(k) = offset + k;

        % --- Update arm plot ---
        cla(ax_arm); hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
        axis(ax_arm,[-lim lim -lim lim]);

        % Workspace boundary
        th_c = linspace(0,2*pi,300);
        plot(ax_arm,(sum(L))*cos(th_c),(sum(L))*sin(th_c),'k--','LineWidth',0.8);

        % Arm
        draw_arm(ax_arm, joints, col);

        % Manipulability ellipse
        draw_ellipse(ax_arm, J, ee, w);

        % Title
        sing_warn = '';
        if w < 0.02, sing_warn = '  ✖ SINGULAR'; end
        if w < 0.08 && w >= 0.02, sing_warn = '  ⚠ near singular'; end
        title(ax_arm, sprintf('%s\nw = %.5f%s', seq.name, w, sing_warn),...
            'FontSize',11,'FontWeight','bold','Color', col);
        xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');

        % --- Update det plot ---
        plot(ax_det, step_ids(1:k), det_hist(1:k), '-','Color',col,'LineWidth',2);
        yline(ax_det, 0,'k--','LineWidth',1);
        if k == n_steps
            xline(ax_det, offset+n_steps,'--','Color',col,'LineWidth',1.2);
            text(ax_det, offset+n_steps/2, max(det_hist)*0.85,...
                sprintf('Seq %d',seq_idx),'Color',col,'FontSize',8,...
                'HorizontalAlignment','center');
        end

        % --- Update w plot ---
        plot(ax_w, step_ids(1:k), w_hist(1:k), '-','Color',col,'LineWidth',2);
        yline(ax_w, 0,'k--','LineWidth',1);
        yline(ax_w, 0.05,'--','Color',[1 0.6 0],'LineWidth',1);

        % --- Update velocity amplification plot ---
        plot(ax_vel, step_ids(1:k), vel_hist(1:k), '-','Color',col,'LineWidth',2);

        % --- Info panel ---
        info_h.String = build_info(seq, q_k, w, d, dq_mag, s_ease);

        drawnow;
        pause(0.025);
    end

    % Mark the singular end-point
    scatter(ax_det, offset+n_steps, det_hist(end), 80, col,...
        'filled','Marker','v');
    scatter(ax_w,   offset+n_steps, w_hist(end),   80, col,...
        'filled','Marker','v');

    % Brief pause between sequences
    pause(0.6);
end

% Final annotations
yline(ax_w, 0.05, '--', 'Color',[1 0.6 0], 'LineWidth',1.5,...
    'Label','⚠ threshold (w=0.05)','LabelHorizontalAlignment','left',...
    'FontSize',8);

% Add legend
legend(ax_det,...
    {'Type 1: Fully Extended','','Type 2: Elbow Lock','','Type 3: Wrist Aligned'},...
    'Location','northeast','FontSize',8);

sgtitle('Singularity Detection  |  3-DOF Planar Arm',...
    'FontSize',13,'FontWeight','bold');

fprintf('\nAnimation complete.\n');
fprintf('Key takeaway: as w → 0, joint velocity amplification → ∞\n');
fprintf('This is why real robots avoid singular configurations!\n');

% =========================================================================
%  HELPER FUNCTIONS
% =========================================================================
function [joints, ee] = fk_planar(theta, L)
    n = length(L);
    joints = zeros(n+1,2);
    cum = 0;
    for i = 1:n
        cum = cum + theta(i);
        joints(i+1,:) = joints(i,:) + L(i)*[cos(cum),sin(cum)];
    end
    ee = joints(end,:);
end

function J = planar_jacobian(theta, L)
    n = length(L);
    J = zeros(2,n);
    [joints, ee] = fk_planar(theta, L);
    for i = 1:n
        r = ee - joints(i,:);
        J(:,i) = [-r(2); r(1)];
    end
end

function draw_arm(ax, joints, col)
    for k = 1:size(joints,1)-1
        p1=joints(k,:); p2=joints(k+1,:);
        plot(ax,[p1(1),p2(1)],[p1(2),p2(2)],'-','Color',col,'LineWidth',5);
    end
    for k = 1:size(joints,1)
        scatter(ax,joints(k,1),joints(k,2),55,...
            'MarkerFaceColor',[0.15 0.15 0.15],'MarkerEdgeColor','k');
    end
    % base marker
    scatter(ax,0,0,80,'k','filled','Marker','square');
end

function draw_ellipse(ax, J, ee, w)
    JJt = J*J';
    if w < 1e-6, return; end
    [V,D] = eig(JJt);
    sa    = sqrt(max(diag(D),0));
    scale = min(0.5/max(sa+1e-8), 3.0);   % cap scale
    th    = linspace(0,2*pi,120);
    ell   = V * diag(sa*scale) * [cos(th); sin(th)];

    % Colour by manipulability
    if w < 0.02
        ec = [0.9 0.1 0.1];
    elseif w < 0.08
        ec = [1.0 0.6 0.0];
    else
        ec = [0.2 0.7 0.2];
    end
    fill(ax, ee(1)+ell(1,:), ee(2)+ell(2,:), ec,...
        'FaceAlpha',0.30,'EdgeColor',ec,'LineWidth',1.8);

    % Principal axes
    for k = 1:2
        v = V(:,k)*sa(k)*scale;
        quiver(ax,ee(1),ee(2),v(1),v(2),...
            'Color',ec,'LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
        quiver(ax,ee(1),ee(2),-v(1),-v(2),...
            'Color',ec,'LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
    end
end

function str = build_info(seq, q, w, d, dq_mag, progress)
    str = sprintf([...
        ' %s\n',...
        ' Progress: %.0f%%\n\n',...
        ' Joint Angles:\n',...
        '   θ1 = %+7.2f°\n',...
        '   θ2 = %+7.2f°\n',...
        '   θ3 = %+7.2f°\n\n',...
        ' Manipulability:\n',...
        '   w  = %.6f\n',...
        '   det= %.6f\n\n',...
        ' Velocity Amplification:\n',...
        '   ||Δθ|| = %.2f\n',...
        '   (for unit EE motion)\n\n',...
        ' Explanation:\n'],...
        seq.name, progress*100,...
        rad2deg(q(1)),rad2deg(q(2)),rad2deg(q(3)),...
        w, d, min(dq_mag,999));
    for i = 1:length(seq.desc)
        str = [str, sprintf('  • %s\n', seq.desc{i})]; %#ok<AGROW>
    end
end
