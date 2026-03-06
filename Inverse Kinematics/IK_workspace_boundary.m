% =========================================================================
% IK WORKSPACE BOUNDARY — Singularities, Boundaries & Unreachable Targets
% =========================================================================
% Demonstrates what happens to IK solutions at and beyond the workspace
% boundary of a 3-DOF planar arm.
%
% KEY CONCEPTS SHOWN:
%   1. UNREACHABLE targets  — outside the outer boundary (r > L1+L2+L3)
%                           — inside the inner boundary (r < |L1-L2-L3|)
%   2. BOUNDARY targets     — arm fully extended or fully folded
%   3. NEAR-SINGULAR configs— det(J·Jᵀ) → 0 as target approaches boundary
%   4. GRACEFUL FALLBACK    — damped least-squares IK instead of crashing
%
% TWO INTERACTION MODES:
%   Mode 1 — CLICK  : click anywhere on the workspace plot to place target
%   Mode 2 — TYPED  : enter [x, y] coordinates manually
%
% The manipulability index w = sqrt(det(J·Jᵀ)) is shown live so students
% can watch it collapse to zero near singularities.
%
% Robot: 3-DOF planar arm  L = [1.0, 0.8, 0.5] m
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  IK Workspace Boundary Explorer  |  3-DOF Planar Arm\n');
fprintf('============================================================\n\n');

% --- Robot parameters ---
L       = [1.0, 0.8, 0.5];     % link lengths (m)
n       = length(L);
R_max   = sum(L);               % outer boundary radius
R_min   = max(0, L(1)-L(2)-L(3)); % inner boundary radius (0 for this config)

% --- IK solver settings ---
alpha_gain = 0.3;               % step size for Jacobian update
lambda     = 0.05;              % damping factor for damped least-squares
tol        = 1e-3;              % convergence tolerance
max_iter   = 300;               % max iterations

fprintf('Arm link lengths: L = [%.1f, %.1f, %.1f] m\n', L(1),L(2),L(3));
fprintf('Outer workspace boundary: %.2f m\n', R_max);
fprintf('Inner workspace boundary: %.2f m\n', R_min);
fprintf('\nManipulability  w = sqrt(det(J·Jᵀ))\n');
fprintf('  w > 0.1  →  Well-conditioned\n');
fprintf('  w < 0.05 →  Near singular  ⚠\n');
fprintf('  w ≈ 0    →  Singular (boundary) ✖\n\n');

% -------------------------------------------------------------------------
%  Build the figure
% -------------------------------------------------------------------------
fig = figure('Name','IK Workspace Boundary Explorer','Color','w',...
    'Position',[60 60 1200 640]);

% Main workspace axes
ax_main = axes('Parent',fig,'Position',[0.04 0.12 0.54 0.82]);
hold(ax_main,'on'); grid(ax_main,'on'); axis(ax_main,'equal');
lim = R_max + 0.4;
axis(ax_main,[-lim lim -lim lim]);
xlabel(ax_main,'X (m)','FontSize',11);
ylabel(ax_main,'Y (m)','FontSize',11);
title(ax_main,'Click to place target  |  or use Mode 2 for typed input',...
    'FontSize',11,'FontWeight','bold');

% Draw workspace boundaries
draw_workspace_boundaries(ax_main, R_max, R_min, L);

% Info panel axes (right side)
ax_info = axes('Parent',fig,'Position',[0.61 0.12 0.37 0.82]);
axis(ax_info,'off');
info_txt = text(ax_info, 0.05, 0.97, build_idle_string(L, R_max),...
    'Units','normalized','VerticalAlignment','top',...
    'FontName','Courier','FontSize',9.5,'Interpreter','none',...
    'BackgroundColor',[0.97 0.97 0.97],'EdgeColor',[0.7 0.7 0.7],...
    'Margin',8);

% Interaction mode selector
uicontrol('Style','text','Units','normalized',...
    'Position',[0.04 0.02 0.12 0.05],...
    'String','Input Mode:','FontSize',10,'FontWeight','bold',...
    'HorizontalAlignment','right','BackgroundColor','w');
mode_btn = uicontrol('Style','popupmenu','Units','normalized',...
    'Position',[0.17 0.03 0.18 0.04],...
    'String',{'Mode 1 — Click','Mode 2 — Type coordinates'},...
    'FontSize',10,'Callback',@mode_changed);

% Type-input controls (hidden initially)
uicontrol('Style','text','Units','normalized',...
    'Position',[0.36 0.025 0.05 0.04],...
    'String','x:','FontSize',11,'BackgroundColor','w');
x_edit = uicontrol('Style','edit','Units','normalized',...
    'Position',[0.41 0.03 0.07 0.04],...
    'String','0.8','FontSize',11,'Visible','off');
uicontrol('Style','text','Units','normalized',...
    'Position',[0.49 0.025 0.05 0.04],...
    'String','y:','FontSize',11,'BackgroundColor','w');
y_edit = uicontrol('Style','edit','Units','normalized',...
    'Position',[0.54 0.03 0.07 0.04],...
    'String','0.5','FontSize',11,'Visible','off');
solve_btn = uicontrol('Style','pushbutton','Units','normalized',...
    'Position',[0.62 0.03 0.12 0.045],...
    'String','Solve IK','FontSize',10,'FontWeight','bold',...
    'BackgroundColor',[0.2 0.6 0.9],'ForegroundColor','w',...
    'Visible','off','Callback',@solve_typed);

% Set up click interaction
set(fig,'WindowButtonDownFcn',@on_click);

% State shared across callbacks
current_mode = 1;
h_arm        = [];     % handle to arm plot objects
h_target     = [];     % handle to target marker

% -------------------------------------------------------------------------
%  Callback: mode changed
% -------------------------------------------------------------------------
    function mode_changed(~,~)
        current_mode = mode_btn.Value;
        if current_mode == 1
            x_edit.Visible  = 'off';
            y_edit.Visible  = 'off';
            solve_btn.Visible = 'off';
            title(ax_main,...
                'Click to place target  |  or use Mode 2 for typed input',...
                'FontSize',11,'FontWeight','bold');
        else
            x_edit.Visible  = 'on';
            y_edit.Visible  = 'on';
            solve_btn.Visible = 'on';
            title(ax_main,...
                'Mode 2: Enter coordinates and press Solve IK',...
                'FontSize',11,'FontWeight','bold');
        end
    end

% -------------------------------------------------------------------------
%  Callback: mouse click on axes
% -------------------------------------------------------------------------
    function on_click(~,~)
        if current_mode ~= 1, return; end
        cp = get(ax_main,'CurrentPoint');
        xd = cp(1,1);  yd = cp(1,2);
        if xd < -lim || xd > lim || yd < -lim || yd > lim, return; end
        solve_and_draw(xd, yd);
    end

% -------------------------------------------------------------------------
%  Callback: typed solve button
% -------------------------------------------------------------------------
    function solve_typed(~,~)
        xd = str2double(x_edit.String);
        yd = str2double(y_edit.String);
        if isnan(xd) || isnan(yd)
            errordlg('Please enter valid numbers for x and y.');
            return;
        end
        solve_and_draw(xd, yd);
    end

% -------------------------------------------------------------------------
%  Core function: solve IK and update the figure
% -------------------------------------------------------------------------
    function solve_and_draw(xd, yd)
        % Clear previous arm drawing
        delete(h_arm);  delete(h_target);
        h_arm = [];

        % --- Classify the target ---
        r    = sqrt(xd^2 + yd^2);
        status = classify_target(r, R_max, R_min);

        % --- Draw target marker (colour-coded by status) ---
        switch status
            case 'reachable'
                t_color = [0.1 0.7 0.1];   marker = 'pentagram';
            case 'boundary'
                t_color = [1.0 0.6 0.0];   marker = 'diamond';
            case 'unreachable'
                t_color = [0.9 0.1 0.1];   marker = 'x';
        end
        h_target = scatter(ax_main, xd, yd, 160, t_color,...
            'filled','Marker',marker,'LineWidth',2,...
            'MarkerEdgeColor','k');

        % --- Attempt IK (always try, even for unreachable) ---
        theta0 = deg2rad([30, -45, 20]);   % initial guess
        [theta_sol, ee_hist, w_hist, converged, n_iters] = ...
            solve_ik_damped(xd, yd, theta0, L, alpha_gain, lambda, tol, max_iter);

        % --- Compute final state ---
        [joints, ee] = fk_planar(theta_sol, L);
        J    = planar_jacobian(theta_sol, L);
        w    = sqrt(max(det(J*J'), 0));
        err  = sqrt((xd-ee(1))^2 + (yd-ee(2))^2);

        % --- Draw arm ---
        arm_color = get_arm_color(w, status);
        h_arm = draw_arm(ax_main, joints, ee, arm_color);

        % --- Draw EE trajectory (path taken by solver) ---
        if size(ee_hist,1) > 2
            h_path = plot(ax_main, ee_hist(:,1), ee_hist(:,2),...
                ':','Color',[0.5 0.5 0.5],'LineWidth',1.2);
            h_arm  = [h_arm; h_path];
        end

        % --- Draw singularity warning indicator ---
        if w < 0.05 && strcmp(status,'reachable')
            h_warn = text(ax_main, joints(end,1)+0.05, joints(end,2)+0.1,...
                '⚠ Near Singular','Color',[1 0.4 0],...
                'FontSize',10,'FontWeight','bold');
            h_arm = [h_arm; h_warn];
        end

        % --- Update info panel ---
        info_txt.String = build_result_string(...
            xd, yd, r, R_max, R_min, status,...
            theta_sol, w, err, converged, n_iters, L);

        % --- Update title ---
        title_str = format_title(status, w, converged, err);
        title(ax_main, title_str, 'FontSize',11,'FontWeight','bold');

        drawnow;
    end

% =========================================================================
%  IK SOLVER — Damped Least Squares (handles singularities gracefully)
% =========================================================================
    function [theta, ee_hist, w_hist, converged, iters] = ...
            solve_ik_damped(xd, yd, theta0, L, gain, lam, tolerance, maxits)

        theta    = theta0;
        ee_hist  = zeros(maxits, 2);
        w_hist   = zeros(maxits, 1);
        converged = false;

        for k = 1:maxits
            [~, ee] = fk_planar(theta, L);
            ee_hist(k,:) = ee;
            J    = planar_jacobian(theta, L);
            w_hist(k) = sqrt(max(det(J*J'),0));

            err_vec = [xd; yd] - ee(:);
            if norm(err_vec) < tolerance
                converged = true;
                iters = k;
                ee_hist = ee_hist(1:k,:);
                w_hist  = w_hist(1:k);
                return;
            end

            % Damped least-squares: J^T (J J^T + λ²I)^{-1}
            J_dls = J' / (J*J' + lam^2 * eye(2));
            dtheta = gain * J_dls * err_vec;
            theta  = theta + dtheta';
        end
        iters    = maxits;
        ee_hist  = ee_hist(1:maxits,:);
        w_hist   = w_hist(1:maxits);
    end

% =========================================================================
%  HELPER FUNCTIONS
% =========================================================================

function status = classify_target(r, R_max, R_min)
    boundary_tol = 0.04;
    if r > R_max + boundary_tol || r < R_min - boundary_tol
        status = 'unreachable';
    elseif abs(r - R_max) < boundary_tol || abs(r - R_min) < boundary_tol
        status = 'boundary';
    else
        status = 'reachable';
    end
end

function col = get_arm_color(w, status)
    if strcmp(status,'unreachable')
        col = [0.9 0.5 0.5];   % pink-red for unreachable
    elseif w < 0.02
        col = [1.0 0.4 0.0];   % orange for singular
    elseif w < 0.08
        col = [1.0 0.8 0.0];   % yellow for near-singular
    else
        col = [0.2 0.5 0.9];   % blue for well-conditioned
    end
end

function h = draw_arm(ax, joints, ~, color)
    h = [];
    % Links
    for k = 1:size(joints,1)-1
        p1 = joints(k,:);   p2 = joints(k+1,:);
        h(end+1) = plot(ax,[p1(1),p2(1)],[p1(2),p2(2)],'-',...
            'Color',color,'LineWidth',5); %#ok<AGROW>
    end
    % Joints
    for k = 1:size(joints,1)
        h(end+1) = scatter(ax, joints(k,1), joints(k,2), 55,...
            'MarkerFaceColor',[0.2 0.2 0.2],'MarkerEdgeColor','k'); %#ok<AGROW>
    end
end

function draw_workspace_boundaries(ax, R_max, R_min, L)
    th = linspace(0,2*pi,400);
    % Outer boundary
    fill(ax, R_max*cos(th), R_max*sin(th),...
        [0.93 0.97 1.0],'EdgeColor','none','FaceAlpha',0.6);
    plot(ax, R_max*cos(th), R_max*sin(th),'b--','LineWidth',1.8);
    text(ax, R_max*cos(0.15), R_max*sin(0.15)+0.1,...
        sprintf('Outer boundary\nr=%.2fm',R_max),...
        'Color',[0 0 0.7],'FontSize',8.5,'FontWeight','bold');
    % Inner boundary
    if R_min > 0.05
        fill(ax, R_min*cos(th), R_min*sin(th),...
            [1 1 1],'EdgeColor','none');
        plot(ax, R_min*cos(th), R_min*sin(th),'r--','LineWidth',1.8);
        text(ax, 0, R_min+0.08, sprintf('Inner\nboundary'),...
            'Color',[0.7 0 0],'FontSize',8.5,'HorizontalAlignment','center');
    end
    % Base
    scatter(ax, 0, 0, 80, 'k', 'filled', 'Marker','square');
    text(ax, 0.06, -0.12,'Base','FontSize',9,'FontWeight','bold');
    % Legend patches
    patch(ax,'XData',[],'YData',[],'FaceColor',[0.1 0.7 0.1],...
        'DisplayName','Reachable target');
    patch(ax,'XData',[],'YData',[],'FaceColor',[1.0 0.6 0.0],...
        'DisplayName','Boundary target');
    patch(ax,'XData',[],'YData',[],'FaceColor',[0.9 0.1 0.1],...
        'DisplayName','Unreachable target');
    plot(ax,NaN,NaN,'-','Color',[0.2 0.5 0.9],'LineWidth',4,...
        'DisplayName','Arm (well-conditioned)');
    plot(ax,NaN,NaN,'-','Color',[1.0 0.8 0.0],'LineWidth',4,...
        'DisplayName','Arm (near singular ⚠)');
    plot(ax,NaN,NaN,'-','Color',[0.9 0.5 0.5],'LineWidth',4,...
        'DisplayName','Arm (unreachable, best effort)');
    legend(ax,'Location','southwest','FontSize',8);
end

function [joints, ee] = fk_planar(theta, L)
    n = length(L);
    joints = zeros(n+1,2);
    cum = 0;
    for i = 1:n
        cum = cum + theta(i);
        joints(i+1,:) = joints(i,:) + L(i)*[cos(cum), sin(cum)];
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

function str = build_idle_string(L, R_max)
    str = sprintf([...
        ' IK WORKSPACE BOUNDARY\n',...
        ' ══════════════════════\n\n',...
        ' Robot: 3-DOF Planar Arm\n',...
        ' Link lengths:\n',...
        '   L1 = %.2f m\n',...
        '   L2 = %.2f m\n',...
        '   L3 = %.2f m\n',...
        '   Total = %.2f m\n\n',...
        ' Solver: Damped Least-Squares\n',...
        '   J_dls = Jᵀ(JJᵀ + λ²I)⁻¹\n',...
        '   Handles singularities\n',...
        '   gracefully without\n',...
        '   numerical blow-up.\n\n',...
        ' Manipulability:\n',...
        '   w = √det(JJᵀ)\n',...
        '   w → 0 at boundary\n\n',...
        ' Colour Guide:\n',...
        '   Blue   = well-conditioned\n',...
        '   Yellow = near singular ⚠\n',...
        '   Red    = unreachable\n\n',...
        ' ── Click the workspace ──\n',...
        '    to place a target\n'],L(1),L(2),L(3),R_max);
end

function str = build_result_string(xd,yd,r,R_max,R_min,...
        status,theta,w,err,converged,iters,L)
    [~, ee] = fk_planar(theta, L);
    if converged
        conv_str = sprintf('YES (%d iters)',iters);
    else
        conv_str = sprintf('NO  (%d iters, max)',iters);
    end
    if w < 0.02
        w_str = sprintf('%.5f  ✖ SINGULAR',w);
    elseif w < 0.08
        w_str = sprintf('%.5f  ⚠ near singular',w);
    else
        w_str = sprintf('%.5f  ✓ OK',w);
    end
    status_symbols = struct('reachable','✓ REACHABLE',...
        'boundary','◆ BOUNDARY','unreachable','✖ UNREACHABLE');
    stat_str = status_symbols.(status);

    str = sprintf([...
        ' TARGET\n',...
        ' ═══════════════════════\n',...
        '  x = %+.4f m\n',...
        '  y = %+.4f m\n',...
        '  r = %.4f m\n\n',...
        ' STATUS: %s\n',...
        '  Outer boundary: %.3fm\n',...
        '  Inner boundary: %.3fm\n\n',...
        ' SOLVER RESULT\n',...
        ' ═══════════════════════\n',...
        '  Converged: %s\n',...
        '  Final error: %.5f m\n\n',...
        ' JOINT ANGLES\n',...
        ' ═══════════════════════\n',...
        '  θ1 = %+7.2f°\n',...
        '  θ2 = %+7.2f°\n',...
        '  θ3 = %+7.2f°\n\n',...
        ' EE (achieved)\n',...
        ' ═══════════════════════\n',...
        '  x = %+.4f m\n',...
        '  y = %+.4f m\n\n',...
        ' MANIPULABILITY\n',...
        ' ═══════════════════════\n',...
        '  w = %s\n'],...
        xd,yd,r,...
        stat_str, R_max, R_min,...
        conv_str, err,...
        rad2deg(theta(1)), rad2deg(theta(2)), rad2deg(theta(3)),...
        ee(1), ee(2),...
        w_str);
end

function str = format_title(status, w, converged, err)
    switch status
        case 'unreachable'
            str = sprintf(...
                'UNREACHABLE target  |  Best-effort solution  |  Error = %.3f m', err);
        case 'boundary'
            str = sprintf(...
                'BOUNDARY target  |  w = %.4f  |  Arm fully extended/folded', w);
        case 'reachable'
            if ~converged
                str = sprintf('IK did not converge  |  Error = %.4f m', err);
            elseif w < 0.05
                str = sprintf(...
                    '⚠ Near Singularity  |  w = %.4f  |  Error = %.5f m', w, err);
            else
                str = sprintf(...
                    '✓ Solved  |  w = %.4f  |  Error = %.5f m', w, err);
            end
    end
end
