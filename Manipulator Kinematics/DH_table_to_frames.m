% =========================================================================
% DH TABLE TO FRAMES - Interactive Parameter Explorer
% =========================================================================
% Visualises exactly what each Denavit-Hartenberg parameter does physically.
%
% The 4 DH parameters for each link:
%   theta (θ) : rotation about z_{i-1} axis  → spins the next frame
%   d         : translation along z_{i-1}     → slides along z
%   a         : translation along x_i         → length of the link
%   alpha (α) : rotation about x_i            → twists the next z-axis
%
% The full transform from frame i-1 to frame i is built in 4 steps:
%   T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
%
% TWO MODES:
%   Mode 1 — AUTO-PLAY : animates each parameter one at a time, showing
%             how the frame moves/rotates as each parameter is applied
%   Mode 2 — SLIDERS   : student controls all 4 parameters live
%
% No toolboxes required.
% =========================================================================

fprintf('============================================================\n');
fprintf('  DH Parameter Explorer\n');
fprintf('============================================================\n\n');
fprintf('  This tool shows what each DH parameter does physically.\n\n');
fprintf('  Mode 1 — AUTO-PLAY : watch each parameter animate\n');
fprintf('  Mode 2 — SLIDERS   : control parameters interactively\n\n');

mode = input('Choose mode (1 or 2): ');
if isempty(mode) || (mode ~= 1 && mode ~= 2)
    mode = 1;
    fprintf('Defaulting to Mode 1 (Auto-Play).\n');
end

% -------------------------------------------------------------------------
%  Default DH parameters to explore (can be changed)
% -------------------------------------------------------------------------
theta_val = 45;    % degrees
d_val     = 0.5;   % metres
a_val     = 1.0;   % metres
alpha_val = 90;    % degrees

if mode == 1
    run_autoplay(theta_val, d_val, a_val, alpha_val);
else
    run_sliders();
end

% =========================================================================
%  MODE 1 — AUTO-PLAY
% =========================================================================
function run_autoplay(theta_deg, d, a, alpha_deg)

    theta = deg2rad(theta_deg);
    alpha = deg2rad(alpha_deg);

    fig = figure('Name','DH Parameters — Auto-Play','Color','w',...
        'Position',[100 80 900 650]);

    steps = 60;   % animation frames per parameter

    param_list = {
        'theta (θ)',  'Rotation about z_{i-1}',  'blue',   theta, 0,   0,   0;
        'd',          'Translation along z_{i-1}','red',    theta, d,   0,   0;
        'a',          'Translation along x_i',    '[0.1 0.6 0.1]', theta, d, a, 0;
        'alpha (α)',  'Rotation about x_i',       '[0.6 0.2 0.8]', theta, d, a, alpha;
    };

    n_params = size(param_list, 1);

    for p = 1:n_params
        p_name  = param_list{p,1};
        p_desc  = param_list{p,2};
        p_color = param_list{p,3};

        % Final values for this step
        th_f = param_list{p,4};
        d_f  = param_list{p,5};
        a_f  = param_list{p,6};
        al_f = param_list{p,7};

        % Start values = final values of PREVIOUS step
        if p == 1
            th_s=0; d_s=0; a_s=0; al_s=0;
        else
            th_s = param_list{p-1,4};
            d_s  = param_list{p-1,5};
            a_s  = param_list{p-1,6};
            al_s = param_list{p-1,7};
        end

        for k = 1:steps
            s  = k / steps;
            th = th_s + s*(th_f - th_s);
            dv = d_s  + s*(d_f  - d_s);
            av = a_s  + s*(a_f  - a_s);
            al = al_s + s*(al_f - al_s);

            T = dh_step_transform(th, dv, av, al);

            clf(fig);
            ax = axes('Parent', fig, 'Position',[0.05 0.12 0.6 0.82]);
            hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
            axis(ax,[-0.5 2.0 -1.2 1.2 -0.3 1.5]);
            view(ax, 35, 25);
            xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

            % Base frame {0}
            plot_frame(ax, eye(4), 'b', 0.3, '{0}');

            % Resulting frame {1}
            if isnumeric(p_color)
                col = p_color;
            else
                col = p_color;
            end
            plot_frame(ax, T, 'r', 0.3, '{1}');

            % Draw the transform path
            draw_dh_path(ax, th, dv, av, al);

            % Title and annotation
            title(ax, sprintf('Step %d/4: Applying  %s', p, p_name),...
                'FontSize', 13, 'FontWeight','bold');

            % Info panel (right side text)
            annotation(fig,'textbox',[0.67 0.55 0.30 0.38],...
                'String', build_info_string(th,dv,av,al,p_name,p_desc,s),...
                'FontSize', 10, 'FontName','Courier',...
                'EdgeColor','k','BackgroundColor',[0.97 0.97 0.97],...
                'Interpreter','none','VerticalAlignment','top');

            % Progress bar
            annotation(fig,'rectangle',[0.67 0.47 0.30*s 0.04],...
                'FaceColor',[0.2 0.6 0.9],'EdgeColor','none');
            annotation(fig,'rectangle',[0.67 0.47 0.30 0.04],...
                'FaceColor','none','EdgeColor','k');
            annotation(fig,'textbox',[0.67 0.41 0.30 0.05],...
                'String',sprintf('Progress: %.0f%%', s*100),...
                'FontSize',9,'EdgeColor','none','HorizontalAlignment','center');

            drawnow;
            pause(0.02);
        end

        % Pause between parameters
        pause(0.4);
    end

    % Final state — show all 4 parameters applied
    clf(fig);
    ax = axes('Parent',fig,'Position',[0.05 0.08 0.92 0.87]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    axis(ax,[-0.5 2.0 -1.2 1.2 -0.3 1.5]);
    view(ax, 35, 25);
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
    T_final = dh_step_transform(theta, d, a, alpha);
    plot_frame(ax, eye(4),   'b', 0.35, '{0}  (base)');
    plot_frame(ax, T_final,  'r', 0.35, '{1}  (result)');
    draw_dh_path(ax, theta, d, a, alpha);
    title(ax, sprintf(['All 4 DH Parameters Applied\n',...
        'θ=%.0f°   d=%.2f   a=%.2f   α=%.0f°'],...
        rad2deg(theta), d, a, rad2deg(alpha)),...
        'FontSize', 13, 'FontWeight','bold');
    fprintf('\nAuto-play complete. Final T:\n'); disp(T_final);
end

% =========================================================================
%  MODE 2 — SLIDERS
% =========================================================================
function run_sliders()

    fig = figure('Name','DH Parameters — Interactive Sliders',...
        'Color','w','Position',[80 60 1050 680]);

    % Axes for 3D plot
    ax = axes('Parent',fig,'Position',[0.04 0.22 0.62 0.73]);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    axis(ax,[-0.5 2.0 -1.2 1.2 -0.3 1.5]);
    view(ax, 35, 25);
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

    % Slider definitions: [label, min, max, default, row]
    slider_cfg = {
        'θ  (deg)',   -180, 180,  45,  1;
        'd   (m)',    -1.0, 1.5,  0.5, 2;
        'a   (m)',     0.0, 2.0,  1.0, 3;
        'α  (deg)',  -180, 180,  90,  4;
    };

    sliders   = gobjects(4,1);
    val_texts = gobjects(4,1);
    colors    = {'b','r',[0.1 0.6 0.1],[0.6 0.2 0.8]};
    param_names = {'θ (z-rotation)','d (z-translation)','a (x-translation)','α (x-rotation)'};

    for i = 1:4
        cfg = slider_cfg{i};
        y   = 0.155 - (i-1)*0.038;

        uicontrol('Style','text','Units','normalized',...
            'Position',[0.04, y+0.005, 0.08, 0.025],...
            'String', cfg{1}, 'FontSize',10,'FontWeight','bold',...
            'HorizontalAlignment','right',...
            'ForegroundColor', colors{i});

        sliders(i) = uicontrol('Style','slider','Units','normalized',...
            'Position',[0.13, y, 0.38, 0.025],...
            'Min',cfg{2},'Max',cfg{3},'Value',cfg{4},...
            'BackgroundColor',[0.85 0.85 0.85]);

        val_texts(i) = uicontrol('Style','text','Units','normalized',...
            'Position',[0.52, y, 0.08, 0.025],...
            'String',sprintf('%.1f', cfg{4}),'FontSize',10,...
            'HorizontalAlignment','left');

        addlistener(sliders(i),'Value','PostSet',@(~,~) update());
    end

    % Info panel
    info_ax = axes('Parent',fig,'Position',[0.68 0.05 0.30 0.90]);
    axis(info_ax,'off');

    update();  % initial draw

    % --- Nested update function ---
    function update()
        th_deg = sliders(1).Value;
        d_val  = sliders(2).Value;
        a_val  = sliders(3).Value;
        al_deg = sliders(4).Value;

        % Update value labels
        val_texts(1).String = sprintf('%.1f°', th_deg);
        val_texts(2).String = sprintf('%.2f m', d_val);
        val_texts(3).String = sprintf('%.2f m', a_val);
        val_texts(4).String = sprintf('%.1f°', al_deg);

        th = deg2rad(th_deg);
        al = deg2rad(al_deg);
        T  = dh_step_transform(th, d_val, a_val, al);

        % Redraw 3D scene
        cla(ax); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
        axis(ax,[-0.5 2.0 -1.2 1.2 -0.3 1.5]);
        plot_frame(ax, eye(4), 'b', 0.30, '{0}');
        plot_frame(ax, T,      'r', 0.30, '{1}');
        draw_dh_path(ax, th, d_val, a_val, al);
        title(ax, 'DH Transform: Drag sliders to explore',...
            'FontSize',12,'FontWeight','bold');

        % Redraw info panel
        cla(info_ax); axis(info_ax,'off');

        % Build T matrix decomposition text
        vals = {th_deg, d_val, a_val, al_deg};
        units= {'°','m','m','°'};
        descs= {'Rot about z_{i-1}','Trans along z_{i-1}',...
                'Trans along x_i', 'Rot about x_i'};

        str = sprintf('DH Transform Matrix T:\n\n');
        for r = 1:4
            str = [str, sprintf(' [%6.3f %6.3f %6.3f %6.3f]\n',...
                T(r,1),T(r,2),T(r,3),T(r,4))];
        end
        str = [str, sprintf('\nParameters:\n')];
        for i2 = 1:4
            str = [str, sprintf('  %s = %.2f%s\n  → %s\n\n',...
                slider_cfg{i2}{1}, vals{i2}, units{i2}, descs{i2})];
        end
        str = [str, sprintf('EE Position:\n  x=%.3f\n  y=%.3f\n  z=%.3f',...
            T(1,4), T(2,4), T(3,4))];

        text(info_ax, 0.02, 0.98, str,...
            'Units','normalized','VerticalAlignment','top',...
            'FontName','Courier','FontSize',9,'Interpreter','none',...
            'BackgroundColor',[0.96 0.96 0.96],...
            'EdgeColor','k','Margin',6);
    end
end

% =========================================================================
%  SHARED HELPER FUNCTIONS
% =========================================================================

function T = dh_step_transform(theta, d, a, alpha)
    % Standard DH: T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end

function draw_dh_path(ax, theta, d, a, alpha)
    % Visualise the 4-step build-up of the DH transform as coloured arrows
    % Step 0: origin
    p0 = [0;0;0];

    % Step 1: after Rot_z(theta) — still at origin, frame has rotated
    % (no translation yet, just show the intermediate z-axis direction)

    % Step 2: after Trans_z(d)
    p2 = [0; 0; d];

    % Step 3: after Trans_x(a) in the rotated frame
    Rz = [cos(theta), -sin(theta), 0;
          sin(theta),  cos(theta), 0;
          0,           0,          1];
    p3 = p2 + Rz * [a; 0; 0];

    % Draw dashed path
    pts = [p0, [0;0;d], p3];
    plot3(ax, pts(1,:), pts(2,:), pts(3,:), 'k--','LineWidth',1.2);

    % Annotate translation steps
    if abs(d) > 0.01
        quiver3(ax, 0,0,0, 0,0,d, 'Color',[0.8 0.2 0.2],...
            'LineWidth',2,'MaxHeadSize',2,'AutoScale','off');
        text(ax, 0.05, 0.05, d/2, sprintf('d=%.2f',d),...
            'Color',[0.8 0.2 0.2],'FontSize',9);
    end
    if abs(a) > 0.01
        v = Rz * [a;0;0];
        quiver3(ax, 0,0,d, v(1),v(2),v(3),'Color',[0.1 0.55 0.1],...
            'LineWidth',2,'MaxHeadSize',2,'AutoScale','off');
        mid = [0;0;d] + v/2;
        text(ax, mid(1)+0.05, mid(2)+0.05, mid(3), sprintf('a=%.2f',a),...
            'Color',[0.1 0.55 0.1],'FontSize',9);
    end
end

function plot_frame(ax, T, color, scale, label)
    R = T(1:3,1:3);
    p = T(1:3,4);
    dirs = R * eye(3);
    axis_colors = {color, color, color};
    axis_labels = {'x','y','z'};
    lw = 2;
    for k = 1:3
        quiver3(ax, p(1),p(2),p(3),...
            dirs(1,k)*scale, dirs(2,k)*scale, dirs(3,k)*scale,...
            'Color',axis_colors{k},'LineWidth',lw,'MaxHeadSize',2,...
            'AutoScale','off');
        tip = p + dirs(:,k)*scale*1.15;
        text(ax, tip(1),tip(2),tip(3), axis_labels{k},...
            'Color',color,'FontWeight','bold','FontSize',9);
    end
    % Frame label at origin
    text(ax, p(1)-0.05, p(2)-0.05, p(3)+scale*0.15, label,...
        'Color',color,'FontWeight','bold','FontSize',11);
end

function str = build_info_string(th, d, a, al, p_name, p_desc, progress)
    T = dh_step_transform(th, d, a, al);
    str = sprintf(['Current parameter:\n  %s\n  (%s)\n\n',...
        'DH values so far:\n',...
        '  theta = %6.2f deg\n',...
        '  d     = %6.3f m\n',...
        '  a     = %6.3f m\n',...
        '  alpha = %6.2f deg\n\n',...
        'Frame {1} origin:\n',...
        '  x = %.4f\n  y = %.4f\n  z = %.4f'],...
        p_name, p_desc,...
        rad2deg(th), d, a, rad2deg(al),...
        T(1,4), T(2,4), T(3,4));
end
