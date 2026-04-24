% =========================================================================
% ROOT LOCUS — Robot Joint Control
% =========================================================================
% Root locus shows how the closed-loop poles move in the s-plane as a
% gain parameter (Kp) increases from 0 to infinity.
%
% Plant model: 1-DOF joint  G(s) = 1 / (J·s² + B·s)
%   Open-loop poles at s=0 (integrator) and s=-B/J (friction pole)
%   No zeros → locus goes to infinity as Kp → ∞
%
% FOUR FEATURES:
%
%   1. POLE MIGRATION — animated dots show how poles travel as Kp grows
%      Right-half plane crossing = instability boundary
%
%   2. STABILITY REGIONS — left half-plane (green) vs right (red)
%      Includes constant damping ratio ζ lines for reference
%
%   3. STEP RESPONSE OVERLAY — click any point on the locus to see
%      the step response for that Kp value
%
%   4. PD ZERO EFFECT — toggle a PD zero (s + z) and watch the locus
%      bend toward the left half-plane — shows why derivative action helps
%
% Requires: Control System Toolbox
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  Root Locus  |  1-DOF Robot Joint\n');
fprintf('============================================================\n\n');

% -------------------------------------------------------------------------
%  Plant parameters
% -------------------------------------------------------------------------
J = 0.5;     % inertia  (kg·m²)
B = 2.0;     % damping  (N·m·s/rad)

s = tf('s');
G = 1 / (J*s^2 + B*s);

p_ol = pole(G);
fprintf('Plant: G(s) = 1 / (%.1f·s² + %.1f·s)\n', J, B);
fprintf('Open-loop poles: s = 0,  s = %.2f\n\n', -B/J);
fprintf('As Kp → ∞:\n');
fprintf('  One pole goes to -∞ along real axis\n');
fprintf('  Other pole breaks away and crosses into RHP → UNSTABLE\n\n');

% Compute breakaway point analytically
% For G = 1/(s(Js+B)), char poly = Js² + Bs + Kp
% Breakaway: dKp/ds = 0 → s = -B/(2J)
s_break = -B/(2*J);
Kp_break = J*s_break^2 + B*abs(s_break);   % gain at breakaway
fprintf('Breakaway point: s = %.4f\n', s_break);
fprintf('Critical gain (instability): Kp_crit ≈ B²/(4J) = %.4f\n\n', B^2/(4*J));

% -------------------------------------------------------------------------
%  Build figure
% -------------------------------------------------------------------------
fig = figure('Name','Root Locus — Robot Joint',...
    'Color','w','Position',[40 40 1380 700]);

% Locus axes (left)
ax_rl = axes('Parent',fig,'Position',[0.04 0.10 0.42 0.83]);
hold(ax_rl,'on'); grid(ax_rl,'on');
xlabel(ax_rl,'Real Axis  σ','FontSize',11);
ylabel(ax_rl,'Imaginary Axis  jω','FontSize',11);
title(ax_rl,'Root Locus  |  Kp sweep + PD zero toggle',...
    'FontSize',11,'FontWeight','bold');

% Step response axes (top-right)
ax_step = axes('Parent',fig,'Position',[0.52 0.42 0.46 0.51]);
hold(ax_step,'on'); grid(ax_step,'on');
xlabel(ax_step,'Time (s)','FontSize',10);
ylabel(ax_step,'θ (rad)','FontSize',10);
title(ax_step,'Step Response  |  Click locus to select Kp',...
    'FontSize',11,'FontWeight','bold');
yline(ax_step,1,'k--','LineWidth',1.5);

% Info axes (bottom-right)
ax_info = axes('Parent',fig,'Position',[0.52 0.06 0.46 0.30]);
axis(ax_info,'off');
info_h = text(ax_info,0.02,0.97,'','Units','normalized',...
    'VerticalAlignment','top','FontName','Courier','FontSize',9.5,...
    'Interpreter','none');

% Toggle button for PD zero
btn_pd = uicontrol('Style','togglebutton','Units','normalized',...
    'Position',[0.52 0.955 0.20 0.038],...
    'String','Add PD Zero  (OFF)','FontSize',10,'FontWeight','bold',...
    'BackgroundColor',[0.85 0.85 0.85],...
    'Callback',@(~,~) toggle_pd());

% Kp slider
uicontrol('Style','text','Units','normalized',...
    'Position',[0.73 0.962 0.06 0.025],...
    'String','Select Kp:','FontSize',9,'BackgroundColor','w');
kp_slider = uicontrol('Style','slider','Units','normalized',...
    'Position',[0.80 0.960 0.14 0.028],...
    'Min',0.1,'Max',50,'Value',10,...
    'BackgroundColor',[0.88 0.88 0.88],...
    'Callback',@(~,~) update_step_response());
kp_label = uicontrol('Style','text','Units','normalized',...
    'Position',[0.95 0.960 0.04 0.025],...
    'String','10.0','FontSize',9,'BackgroundColor','w','FontWeight','bold');

% Kp range for locus
Kp_vec  = [logspace(-2, log10(B^2/(4*J))*0.99, 150), ...
           linspace(B^2/(4*J), B^2/(4*J)*3, 50)];
use_pd  = false;
z_pd    = 3.0;     % PD zero location (s + z_pd) → zero at s = -z_pd

draw_full_locus();
update_step_response();

% =========================================================================
%  DRAW FULL LOCUS
% =========================================================================
    function draw_full_locus()
        cla(ax_rl); hold(ax_rl,'on'); grid(ax_rl,'on');

        % ---- Stability regions ----
        fill(ax_rl,[0 15 15 0],[-12 -12 12 12],...
            [1.0 0.88 0.88],'FaceAlpha',0.35,'EdgeColor','none');
        fill(ax_rl,[-15 0 0 -15],[-12 -12 12 12],...
            [0.88 1.0 0.88],'FaceAlpha',0.25,'EdgeColor','none');
        text(ax_rl,7,10,'UNSTABLE','Color',[0.7 0 0],...
            'FontSize',11,'FontWeight','bold','HorizontalAlignment','center');
        text(ax_rl,-7,10,'STABLE','Color',[0 0.5 0],...
            'FontSize',11,'FontWeight','bold','HorizontalAlignment','center');

        % ---- Constant ζ lines (damping ratio) ----
        zeta_vals = [0.3, 0.5, 0.7, 1.0];
        for zv = zeta_vals
            % Line: angle = acos(ζ) from negative real axis
            ang = acos(zv);
            r   = 10;
            plot(ax_rl, [-r*cos(ang), 0], [ r*sin(ang), 0],...
                ':', 'Color',[0.65 0.65 0.65],'LineWidth',0.8);
            plot(ax_rl, [-r*cos(ang), 0], [-r*sin(ang), 0],...
                ':', 'Color',[0.65 0.65 0.65],'LineWidth',0.8);
            text(ax_rl,-r*cos(ang)-0.2, r*sin(ang)+0.3,...
                sprintf('ζ=%.1f',zv),'FontSize',7.5,...
                'Color',[0.55 0.55 0.55]);
        end
        xline(ax_rl,0,'k-','LineWidth',1.0);
        yline(ax_rl,0,'k-','LineWidth',1.0);

        % ---- Compute and plot locus ----
        if use_pd
            G_loop = (s + z_pd) * G;
            col_rl = [0.85 0.35 0.10];
            locus_label = sprintf('Root Locus  (with PD zero at s=−%.1f)',z_pd);
        else
            G_loop = G;
            col_rl = [0.15 0.45 0.85];
            locus_label = 'Root Locus  (P control only)';
        end

        % Compute closed-loop poles for each Kp
        locus_pts = zeros(length(Kp_vec), 2);   % [n_Kp × 2 poles]
        for ki = 1:length(Kp_vec)
            kp_i = Kp_vec(ki);
            T_cl = feedback(kp_i * G_loop, 1);
            p_cl = pole(T_cl);
            p_cl = sort(p_cl, 'ComparisonMethod','real');
            locus_pts(ki,:) = p_cl(1:2)';
        end

        % Plot locus branches
        for branch = 1:2
            pts = locus_pts(:,branch);
            % Colour by stability
            for ki = 1:length(Kp_vec)-1
                seg_col = col_rl;
                if real(pts(ki)) > 0 || real(pts(ki+1)) > 0
                    seg_col = [0.9 0.2 0.2];
                end
                plot(ax_rl,[real(pts(ki)),real(pts(ki+1))],...
                    [imag(pts(ki)),imag(pts(ki+1))],'-',...
                    'Color',seg_col,'LineWidth',2.5);
            end
        end

        % Direction arrows
        arrow_idxs = round(linspace(20,length(Kp_vec)-5, 6));
        for ki = arrow_idxs
            for branch = 1:2
                p1 = locus_pts(ki,  branch);
                p2 = locus_pts(ki+1,branch);
                quiver(ax_rl,real(p1),imag(p1),...
                    real(p2-p1),imag(p2-p1),...
                    'Color',col_rl,'LineWidth',1.5,...
                    'MaxHeadSize',6,'AutoScale','on','AutoScaleFactor',0.3);
            end
        end

        % Open-loop poles (×)
        p_plant = pole(G);
        if use_pd
            p_plant = pole(G_loop);
        end
        scatter(ax_rl, real(p_plant), imag(p_plant),...
            120,'k','x','LineWidth',3,'SizeData',150,...
            'DisplayName','OL poles (×)');

        % Open-loop zeros (○)
        z_plant = zero(G_loop);
        if ~isempty(z_plant)
            scatter(ax_rl, real(z_plant), imag(z_plant),...
                100,[0.1 0.65 0.25],'o','LineWidth',2.5,...
                'DisplayName','OL zeros (○)');
            text(ax_rl,real(z_plant(1))-0.2,imag(z_plant(1))+0.5,...
                sprintf('z=−%.1f',z_pd),'Color',[0.1 0.65 0.25],...
                'FontSize',9,'FontWeight','bold');
        end

        % Breakaway point
        if ~use_pd
            scatter(ax_rl, s_break, 0, 120, [1 0.6 0],...
                'filled','Marker','square','DisplayName',...
                sprintf('Breakaway  Kp=%.2f',Kp_break));
            text(ax_rl, s_break-0.2, 0.6,...
                sprintf('Breakaway\nKp=%.2f',Kp_break),...
                'Color',[0.8 0.4 0],'FontSize',8.5,'FontWeight','bold');
        end

        % Mark current Kp on locus
        Kp_cur = kp_slider.Value;
        T_cur  = feedback(Kp_cur*G_loop, 1);
        p_cur  = pole(T_cur);
        scatter(ax_rl, real(p_cur), imag(p_cur), 100,...
            'm','filled','MarkerEdgeColor','k',...
            'DisplayName',sprintf('Current Kp=%.1f',Kp_cur));

        legend(ax_rl,'Location','northwest','FontSize',8);
        title(ax_rl, locus_label,'FontSize',11,'FontWeight','bold');
        xlabel(ax_rl,'Real Axis  σ','FontSize',11);
        ylabel(ax_rl,'Imaginary Axis  jω','FontSize',11);
        axis(ax_rl,[-12 6 -10 10]);
    end

% =========================================================================
%  UPDATE STEP RESPONSE for selected Kp
% =========================================================================
    function update_step_response()
        Kp_cur = kp_slider.Value;
        kp_label.String = sprintf('%.1f', Kp_cur);

        if use_pd
            G_loop = (s + z_pd) * G;
        else
            G_loop = G;
        end

        T_cl   = feedback(Kp_cur * G_loop, 1);
        p_cl   = pole(T_cl);
        stable = all(real(p_cl) < 0);

        t_sim = linspace(0, 5, 800);

        cla(ax_step); hold(ax_step,'on'); grid(ax_step,'on');
        yline(ax_step,1,'k--','LineWidth',1.5);

        if stable
            try
                [y,~] = step(T_cl, t_sim);
                col   = get_response_color(Kp_cur);
                plot(ax_step, t_sim, y, '-','Color',col,'LineWidth',2.5);

                % Metrics
                y_ss = y(end);
                [y_max,~] = max(y);
                OS = max(0,(y_max-y_ss)/y_ss*100);
                i10 = find(y>=0.1*y_ss,1); i90=find(y>=0.9*y_ss,1);
                tr  = NaN; if ~isempty(i10)&&~isempty(i90), tr=t_sim(i90)-t_sim(i10); end
                se  = abs(1-y_ss)*100;

                title(ax_step,...
                    sprintf('Step Response  |  Kp=%.1f  |  OS=%.1f%%  tr=%.3fs  SS err=%.2f%%',...
                    Kp_cur, OS, tr, se),...
                    'FontSize',10,'FontWeight','bold');
            catch
                stable = false;
            end
        end

        if ~stable
            t_short = linspace(0,1,200);
            try
                [y,~] = step(T_cl, t_short);
                plot(ax_step, t_short, min(max(y,-3),5),'-',...
                    'Color',[0.9 0.2 0.2],'LineWidth',2);
            catch; end
            text(ax_step,2.5,2,'⚠  UNSTABLE',...
                'Color',[0.9 0.1 0.1],'FontSize',16,'FontWeight','bold',...
                'HorizontalAlignment','center');
            title(ax_step,sprintf('UNSTABLE  |  Kp=%.1f exceeds critical gain',Kp_cur),...
                'FontSize',10,'FontWeight','bold','Color',[0.8 0.1 0.1]);
        end

        xlabel(ax_step,'Time (s)','FontSize',10);
        ylabel(ax_step,'θ (rad)','FontSize',10);

        % Update locus marker + info
        draw_locus_marker(Kp_cur, G_loop);
        update_info(Kp_cur, p_cl, stable, G_loop);
    end

% =========================================================================
%  TOGGLE PD ZERO
% =========================================================================
    function toggle_pd()
        use_pd = logical(btn_pd.Value);
        if use_pd
            btn_pd.String  = sprintf('PD Zero ON  (s+%.1f)',z_pd);
            btn_pd.BackgroundColor = [0.7 0.9 0.7];
        else
            btn_pd.String  = 'Add PD Zero  (OFF)';
            btn_pd.BackgroundColor = [0.85 0.85 0.85];
        end
        draw_full_locus();
        update_step_response();
    end

% =========================================================================
%  HELPERS
% =========================================================================
    function draw_locus_marker(Kp_cur, G_loop)
        T_cur = feedback(Kp_cur*G_loop,1);
        p_cur = pole(T_cur);
        delete(findobj(ax_rl,'Tag','cur_pole'));
        scatter(ax_rl,real(p_cur),imag(p_cur),120,'m','filled',...
            'MarkerEdgeColor','k','Tag','cur_pole');
    end

    function update_info(Kp_cur, p_cl, stable, G_loop)
        Kp_crit = B^2/(4*J);
        if use_pd
            pd_str = sprintf('YES — zero at s=−%.1f',z_pd);
        else
            pd_str = 'NO';
        end

        if stable
            stab_str = '✓ STABLE';
        else
            stab_str = '✖ UNSTABLE';
        end

        str = sprintf([...
            ' Plant G(s) = 1/(%.1fs²+%.1fs)\n\n',...
            ' Selected Kp    = %.2f\n',...
            ' Critical Kp    = %.4f\n',...
            ' Margin         = %.1f%%\n\n',...
            ' Stability: %s\n\n',...
            ' Closed-Loop Poles:\n'],...
            J,B,Kp_cur,Kp_crit,...
            max(0,(Kp_crit-Kp_cur)/Kp_crit*100),...
            stab_str);
        for i=1:length(p_cl)
            pc=p_cl(i);
            if imag(pc)>=0
                str=[str,sprintf('   %+.3f %+.3fi\n',real(pc),imag(pc))]; %#ok<AGROW>
            end
        end
        str=[str,sprintf(['\n PD Zero:  %s\n\n',...
            ' Root Locus Rules:\n',...
            '  • Starts at OL poles (Kp=0)\n',...
            '  • Ends at OL zeros or ∞\n',...
            '  • n−m branches → ∞\n',...
            '  • Breakaway: dKp/ds = 0\n',...
            '  • RHP crossing → instability\n',...
            '  • PD zero bends locus left'],pd_str)];
        info_h.String = str;
    end

    function col = get_response_color(Kp)
        % Colour shifts blue→green→orange as Kp increases
        Kp_crit = B^2/(4*J);
        frac = min(Kp/Kp_crit, 1.0);
        col  = [frac*0.7, 0.45*(1-frac)+0.45, 0.85*(1-frac)];
    end
