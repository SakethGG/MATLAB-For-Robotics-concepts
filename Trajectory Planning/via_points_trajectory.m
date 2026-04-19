% =========================================================================
% VIA-POINTS TRAJECTORY — Multi-Waypoint Motion Planning
% =========================================================================
% Real robot tasks require passing through MULTIPLE waypoints, not just
% moving from A to B. Via-points define intermediate positions the
% end-effector must pass through (or near).
%
% TWO APPROACHES to connect via-points:
%
%   1. CUBIC SPLINE — smooth curve passing EXACTLY through all waypoints.
%      Velocity at each via-point is chosen automatically to ensure
%      continuity of position, velocity and acceleration across segments.
%      Used in: welding, painting, precision assembly.
%
%   2. LSPB (Linear Segments with Parabolic Blends) — straight-line
%      segments connected by parabolic transition zones. Does NOT pass
%      exactly through intermediate points (blends around them).
%      Used in: pick-and-place, fast point-to-point motion.
%
% TWO TASK DEMONSTRATIONS:
%   TASK 1 — PICK-AND-PLACE: lift over an obstacle
%             Waypoints: [start] → [lift] → [over obstacle] → [place]
%             Students see why intermediate via-points are essential.
%
%   TASK 2 — SHAPE DRAWING: trace a triangle or star
%             Waypoints define the vertices; robot must pass through each.
%             Shows cubic spline vs straight-line segment differences.
%
% Robot: 2-DOF planar arm  L = [1.0, 0.8] m
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Via-Points Trajectory  |  Pick-and-Place & Shape Drawing\n');
fprintf('============================================================\n\n');

L = [1.0, 0.8];

fprintf('Choose a task:\n');
fprintf('  1 — Pick-and-Place (lift over obstacle)\n');
fprintf('  2 — Shape Drawing  (trace a triangle)\n');
fprintf('  3 — Run both tasks sequentially\n');
task = input('Enter choice [1/2/3, default 3]: ');
if isempty(task), task = 3; end

if task == 1 || task == 3
    run_pick_and_place(L);
end
if task == 2 || task == 3
    run_shape_drawing(L);
end

% =========================================================================
%  TASK 1: PICK-AND-PLACE
% =========================================================================
function run_pick_and_place(L)

    fprintf('\n══════════════════════════════════════════════\n');
    fprintf('  TASK 1: Pick-and-Place Over an Obstacle\n');
    fprintf('══════════════════════════════════════════════\n\n');

    % Obstacle definition
    obs_x = [0.55, 0.85];   % x range
    obs_y = [0.00, 0.45];   % y range (height)

    % Cartesian via-points [x, y] for EE
    waypoints = [0.20,  0.10;    % 1: start (pick position)
                 0.20,  0.55;    % 2: lift above start
                 0.50,  0.65;    % 3: over obstacle
                 0.90,  0.55;    % 4: descend on other side
                 0.90,  0.10];   % 5: place position

    n_wp = size(waypoints,1);
    T_seg = 1.2;    % time per segment (s)
    dt    = 0.02;

    fprintf('Via-points (EE Cartesian):\n');
    for i = 1:n_wp
        labels = {'Pick','Lift up','Over obstacle','Descend','Place'};
        fprintf('  WP%d: (%.2f, %.2f)  — %s\n',...
            i, waypoints(i,1), waypoints(i,2), labels{i});
    end

    % Solve IK for each waypoint
    q_wp = zeros(n_wp, 2);
    q_wp(1,:) = ik_2dof(waypoints(1,:), L, [0.3, -0.5]);
    for i = 2:n_wp
        q_wp(i,:) = ik_2dof(waypoints(i,:), L, q_wp(i-1,:));
    end

    fprintf('\nJoint angles at via-points:\n');
    for i=1:n_wp
        fprintf('  WP%d: θ1=%.1f°  θ2=%.1f°\n',...
            i, rad2deg(q_wp(i,1)), rad2deg(q_wp(i,2)));
    end

    % ---- Generate joint-space cubic spline trajectory ----
    t_wp   = (0:n_wp-1) * T_seg;
    t_all  = 0:dt:(n_wp-1)*T_seg;

    [q_traj, qd_traj] = cubic_spline_traj(t_wp, q_wp, t_all);

    % Compute EE path
    N = length(t_all);
    ee_path = zeros(N,2);
    for k=1:N
        [~,ee_path(k,:)] = fk_2dof(q_traj(k,:), L);
    end

    % ---- Also compute naive direct path (no via-points, just A→B) ----
    q_direct = zeros(N,2);
    s_vec = (t_all - t_all(1))/(t_all(end)-t_all(1));
    s_smooth = (1-cos(pi*s_vec))/2;
    for k=1:N
        q_direct(k,:) = q_wp(1,:) + s_smooth(k)*(q_wp(end,:)-q_wp(1,:));
    end
    ee_direct = zeros(N,2);
    for k=1:N
        [~,ee_direct(k,:)] = fk_2dof(q_direct(k,:),L);
    end

    % ---- Plot ----
    fig = figure('Name','Via-Points: Pick-and-Place','Color','w',...
        'Position',[50 50 1300 580]);

    ax_arm = subplot(1,3,[1,2]);
    hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
    axis(ax_arm,[-0.2 1.4 -0.15 1.1]);
    xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
    title(ax_arm,'Pick-and-Place Over Obstacle','FontSize',12,'FontWeight','bold');

    % Obstacle
    fill(ax_arm,[obs_x(1),obs_x(2),obs_x(2),obs_x(1)],...
        [0,0,obs_y(2),obs_y(2)],[0.6 0.6 0.6],'EdgeColor','k','LineWidth',1.5);
    text(ax_arm,mean(obs_x),obs_y(2)+0.04,'Obstacle',...
        'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');

    % Full paths
    plot(ax_arm, ee_path(:,1),   ee_path(:,2),   '-','Color',[0.15 0.55 0.85],...
        'LineWidth',2.5,'DisplayName','Via-point path');
    plot(ax_arm, ee_direct(:,1), ee_direct(:,2), '--','Color',[0.85 0.35 0.10],...
        'LineWidth',2,'DisplayName','Direct path (no via-points)');

    % Via-points
    scatter(ax_arm, waypoints(:,1), waypoints(:,2), 100,...
        'g','filled','Marker','diamond','MarkerEdgeColor','k',...
        'DisplayName','Via-points');
    for i=1:n_wp
        text(ax_arm, waypoints(i,1)+0.03, waypoints(i,2)+0.04,...
            sprintf('WP%d',i),'FontSize',9,'FontWeight','bold','Color',[0.1 0.5 0.1]);
    end

    % Check obstacle collision for direct path
    in_obs = ee_direct(:,1)>=obs_x(1) & ee_direct(:,1)<=obs_x(2) & ...
             ee_direct(:,2)>=0        & ee_direct(:,2)<=obs_y(2);
    if any(in_obs)
        scatter(ax_arm, ee_direct(in_obs,1), ee_direct(in_obs,2),...
            20,'r','filled','DisplayName','Collision! ✖');
        text(ax_arm,mean(obs_x)-0.05,obs_y(2)/2,...
            '✖ COLLISION','Color','r','FontWeight','bold','FontSize',11,...
            'HorizontalAlignment','center');
    end
    legend(ax_arm,'Location','northwest','FontSize',9);

    % Joint angle profile
    ax_q = subplot(1,3,3);
    hold(ax_q,'on'); grid(ax_q,'on');
    plot(ax_q, t_all, rad2deg(q_traj(:,1)), '-','Color',[0.2 0.5 0.9],...
        'LineWidth',2,'DisplayName','θ₁');
    plot(ax_q, t_all, rad2deg(q_traj(:,2)), '-','Color',[0.9 0.4 0.1],...
        'LineWidth',2,'DisplayName','θ₂');
    for i=1:n_wp
        xline(ax_q,(i-1)*T_seg,'k--','LineWidth',1,...
            'Label',sprintf('WP%d',i),'FontSize',8,...
            'LabelVerticalAlignment','bottom');
    end
    xlabel(ax_q,'Time (s)'); ylabel(ax_q,'Angle (°)');
    title(ax_q,'Joint Angles','FontSize',11,'FontWeight','bold');
    legend(ax_q,'Location','best','FontSize',10);

    % ---- Animate ----
    for k = 1:2:N
        [joints,~] = fk_2dof(q_traj(k,:), L);
        pts = [zeros(1,2); joints(1,:); joints(2,:)];

        % Remove previous arm
        delete(findobj(ax_arm,'Tag','arm'));
        for seg=1:2
            plot(ax_arm,[pts(seg,1),pts(seg+1,1)],[pts(seg,2),pts(seg+1,2)],...
                '-','Color',[0.15 0.45 0.85],'LineWidth',5,'Tag','arm');
        end
        for seg=1:3
            scatter(ax_arm,pts(seg,1),pts(seg,2),60,...
                'MarkerFaceColor','k','MarkerEdgeColor','k','Tag','arm');
        end
        xline(ax_q, t_all(k),'r-','LineWidth',1.5,'Tag','arm');
        drawnow; pause(0.01);
    end
    sgtitle('Via-Points Trajectory  |  Pick-and-Place',...
        'FontSize',13,'FontWeight','bold');
end

% =========================================================================
%  TASK 2: SHAPE DRAWING
% =========================================================================
function run_shape_drawing(L)

    fprintf('\n══════════════════════════════════════════════\n');
    fprintf('  TASK 2: Shape Drawing — Triangle\n');
    fprintf('══════════════════════════════════════════════\n\n');

    % Triangle vertices (EE Cartesian positions)
    cx = 0.80;  cy = 0.60;  r = 0.35;   % centre and radius
    angles_tri = [90, 210, 330, 90];     % close the triangle
    waypoints  = [cx + r*cosd(angles_tri)', cy + r*sind(angles_tri)'];

    n_wp  = size(waypoints,1);
    T_seg = 1.5;
    dt    = 0.02;

    fprintf('Triangle vertices (EE Cartesian):\n');
    for i=1:n_wp
        fprintf('  V%d: (%.3f, %.3f)\n', i, waypoints(i,1), waypoints(i,2));
    end

    % IK for each vertex
    q_wp = zeros(n_wp,2);
    q_wp(1,:) = ik_2dof(waypoints(1,:), L, [0.5, -1.0]);
    for i=2:n_wp
        q_wp(i,:) = ik_2dof(waypoints(i,:), L, q_wp(i-1,:));
    end

    t_wp  = (0:n_wp-1)*T_seg;
    t_all = 0:dt:(n_wp-1)*T_seg;
    N     = length(t_all);

    % Cubic spline (smooth, passes exactly through vertices)
    [q_spline, ~] = cubic_spline_traj(t_wp, q_wp, t_all);

    % LSPB (blended, rounds the corners)
    q_lspb = lspb_multipoint(t_wp, q_wp, t_all);

    % Compute EE paths
    ee_spline = zeros(N,2);
    ee_lspb   = zeros(N,2);
    for k=1:N
        [~,ee_spline(k,:)] = fk_2dof(q_spline(k,:), L);
        [~,ee_lspb(k,:)  ] = fk_2dof(q_lspb(k,:),   L);
    end

    % ---- Figure ----
    fig = figure('Name','Via-Points: Shape Drawing','Color','w',...
        'Position',[60 60 1300 560]);

    c_spl = [0.15 0.55 0.85];
    c_lsp = [0.85 0.40 0.10];

    for col=1:2
        ax = subplot(1,2,col);
        hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
        axis(ax,[0.2 1.4 0.1 1.1]);
        xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');

        if col==1, col_str='Cubic Spline'; col_c=c_spl; ee_p=ee_spline; q_p=q_spline;
        else,      col_str='LSPB';         col_c=c_lsp; ee_p=ee_lspb;   q_p=q_lspb; end

        % Target shape (ideal triangle)
        plot(ax, waypoints(:,1), waypoints(:,2), 'k--','LineWidth',1.5,...
            'DisplayName','Ideal triangle');
        scatter(ax, waypoints(:,1), waypoints(:,2), 100,...
            'k','filled','Marker','diamond','DisplayName','Vertices');

        % EE path
        plot(ax, ee_p(:,1), ee_p(:,2), '-','Color',col_c,...
            'LineWidth',2.5,'DisplayName',col_str);

        % Vertex labels
        for i=1:n_wp
            text(ax,waypoints(i,1)+0.03,waypoints(i,2)+0.03,...
                sprintf('V%d',i),'FontSize',10,'FontWeight','bold');
        end
        legend(ax,'Location','best','FontSize',9);
        title(ax, col_str,'FontSize',12,'FontWeight','bold','Color',col_c);
    end

    sgtitle('Via-Points: Shape Drawing  |  Cubic Spline vs LSPB',...
        'FontSize',13,'FontWeight','bold');

    % ---- Animate both simultaneously ----
    for k=1:3:N
        for col=1:2
            ax = subplot(1,2,col);
            if col==1, q_p=q_spline; col_c=c_spl;
            else,      q_p=q_lspb;   col_c=c_lsp; end
            [joints,~] = fk_2dof(q_p(k,:),L);
            pts = [zeros(1,2); joints(1,:); joints(2,:)];
            delete(findobj(ax,'Tag','arm'));
            for seg=1:2
                plot(ax,[pts(seg,1),pts(seg+1,1)],[pts(seg,2),pts(seg+1,2)],...
                    '-','Color',col_c,'LineWidth',4,'Tag','arm');
            end
            for seg=1:3
                scatter(ax,pts(seg,1),pts(seg,2),50,...
                    'MarkerFaceColor','k','MarkerEdgeColor','k','Tag','arm');
            end
        end
        drawnow; pause(0.01);
    end
    fprintf('Shape drawing complete.\n');
end

% =========================================================================
%  TRAJECTORY GENERATION FUNCTIONS
% =========================================================================
function [q_out, qd_out] = cubic_spline_traj(t_wp, q_wp, t_out)
    % Natural cubic spline in joint space
    n_joints = size(q_wp,2);
    q_out    = zeros(length(t_out), n_joints);
    qd_out   = zeros(length(t_out), n_joints);
    for j = 1:n_joints
        pp = spline(t_wp, q_wp(:,j)');
        q_out(:,j)  = ppval(pp, t_out)';
        % Derivative
        pp_d = pp;
        pp_d.coefs = pp.coefs(:,1:end-1) .* repmat((size(pp.coefs,2)-1:-1:1),size(pp.coefs,1),1);
        pp_d.order = pp.order-1;
        qd_out(:,j) = ppval(pp_d, t_out)';
    end
end

function q_out = lspb_multipoint(t_wp, q_wp, t_out)
    % LSPB between consecutive via-points
    n_joints = size(q_wp,2);
    n_segs   = length(t_wp)-1;
    q_out    = zeros(length(t_out), n_joints);

    for seg = 1:n_segs
        t0 = t_wp(seg);   tf = t_wp(seg+1);
        mask = t_out >= t0 & t_out <= tf;
        t_seg = t_out(mask) - t0;
        T_seg = tf - t0;
        tb    = 0.25*T_seg;

        for j = 1:n_joints
            q0j = q_wp(seg,j);   qfj = q_wp(seg+1,j);
            V   = (qfj-q0j)/(T_seg-tb);
            q_s = zeros(size(t_seg));
            for ki=1:length(t_seg)
                tk=t_seg(ki);
                if tk<=tb
                    q_s(ki) = q0j + V/(2*tb)*tk^2;
                elseif tk<=T_seg-tb
                    q_s(ki) = q0j + V*(tk-tb/2);
                else
                    q_s(ki) = qfj - V/(2*tb)*(T_seg-tk)^2;
                end
            end
            q_out(mask,j) = q_s;
        end
    end
end

function [joints, ee] = fk_2dof(q, L)
    joints = zeros(2,2);
    joints(1,:) = L(1)*[cos(q(1)),sin(q(1))];
    joints(2,:) = joints(1,:)+L(2)*[cos(q(1)+q(2)),sin(q(1)+q(2))];
    ee = joints(end,:);
end

function q = ik_2dof(ee_d, L, q0)
    q = q0;
    for iter=1:150
        [~,ee] = fk_2dof(q,L);
        err = ee_d(:)-ee(:);
        if norm(err)<5e-5, break; end
        cum = q(1); J = zeros(2,2);
        p  = [0;0];
        origins = [[0;0], L(1)*[cos(cum);sin(cum)]];
        ee_v = [ee(1);ee(2)];
        for i=1:2
            r = ee_v - origins(:,i);
            J(:,i) = [-r(2);r(1)];
        end
        dq = 0.5*(J'/(J*J'+0.01^2*eye(2)))*err;
        q  = q+dq';
    end
end
