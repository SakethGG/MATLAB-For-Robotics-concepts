% =========================================================================
% PID GAIN TUNING — Interactive Explorer
% =========================================================================
% Plant model: 1-DOF robot joint with motor inertia and viscous friction
%
%   J·θ̈ + B·θ̇ = τ      (torque = inertia × accel + friction × vel)
%
%   Transfer function:  G(s) = 1 / (J·s² + B·s)
%
%   PID Controller:     C(s) = Kp + Ki/s + Kd·s
%                            = (Kd·s² + Kp·s + Ki) / s
%
%   Closed-loop:        T(s) = C(s)·G(s) / (1 + C(s)·G(s))
%
% FEATURES:
%   ✓ Sliders for Kp, Ki, Kd — step response updates live
%   ✓ Live metrics: rise time, overshoot, settling time, SS error
%   ✓ "Build-up" panel: shows effect of each gain independently
%     P only → PD → PID so students see exactly what each term adds
%   ✓ Instability warning when closed-loop poles enter RHP
%   ✓ Pole-zero map updates live alongside step response
%
% Requires: Control System Toolbox
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  PID Gain Tuning  |  Interactive Explorer\n');
fprintf('============================================================\n\n');

% -------------------------------------------------------------------------
%  Plant parameters  (1-DOF joint: J·θ̈ + B·θ̇ = τ)
% -------------------------------------------------------------------------
J = 0.5;     % joint inertia  (kg·m²)
B = 2.0;     % viscous friction (N·m·s/rad)

fprintf('Plant: G(s) = 1/(J·s² + B·s)\n');
fprintf('  J = %.2f kg·m²   B = %.2f N·m·s/rad\n\n', J, B);
fprintf('Open-loop poles at s = 0  and  s = -B/J = %.2f\n\n', -B/J);

% Plant transfer function
s  = tf('s');
G  = 1 / (J*s^2 + B*s);

% Simulation time
t_sim = linspace(0, 3.0, 600);

% -------------------------------------------------------------------------
%  Build interactive figure
% -------------------------------------------------------------------------
fig = figure('Name','PID Gain Tuning — Interactive Explorer',...
    'Color','w','Position',[30 30 1380 740]);

% ---- Axes layout ----
ax_step = axes('Parent',fig,'Position',[0.05 0.38 0.42 0.55]);
hold(ax_step,'on'); grid(ax_step,'on');
xlabel(ax_step,'Time (s)','FontSize',10);
ylabel(ax_step,'Joint Angle (rad)','FontSize',10);
title(ax_step,'Step Response','FontSize',11,'FontWeight','bold');
yline(ax_step,1,'k--','LineWidth',1.5,'Label','Setpoint','FontSize',9,...
    'LabelVerticalAlignment','bottom');

ax_pz = axes('Parent',fig,'Position',[0.54 0.38 0.22 0.55]);
hold(ax_pz,'on'); grid(ax_pz,'on'); axis(ax_pz,'equal');
xlabel(ax_pz,'Real','FontSize',10); ylabel(ax_pz,'Imag','FontSize',10);
title(ax_pz,'Pole-Zero Map','FontSize',11,'FontWeight','bold');
xline(ax_pz,0,'k-','LineWidth',1.2);
yline(ax_pz,0,'k-','LineWidth',1.2);
text(ax_pz,-0.3,0,'Stable ←','FontSize',8,'Color',[0.5 0.5 0.5],...
    'HorizontalAlignment','right');

ax_build = axes('Parent',fig,'Position',[0.79 0.38 0.19 0.55]);
hold(ax_build,'on'); grid(ax_build,'on');
xlabel(ax_build,'Time (s)','FontSize',9);
ylabel(ax_build,'Response','FontSize',9);
title(ax_build,'Build-Up: P → PD → PID','FontSize',10,'FontWeight','bold');
yline(ax_build,1,'k--','LineWidth',1.2);

% Info panel
ax_info = axes('Parent',fig,'Position',[0.54 0.04 0.44 0.28]);
axis(ax_info,'off');
info_h = text(ax_info,0.02,0.97,'','Units','normalized',...
    'VerticalAlignment','top','FontName','Courier','FontSize',9.5,...
    'Interpreter','none');

% -------------------------------------------------------------------------
%  Slider panel (bottom-left)
% -------------------------------------------------------------------------
slider_cfg = {
%   label     min    max    default  row
    'Kp',     0,     200,   40,      1;
    'Ki',     0,     100,   8,       2;
    'Kd',     0,     20,    4,       3;
};

sliders   = gobjects(3,1);
val_labels = gobjects(3,1);
slider_colors = {[0.15 0.50 0.85], [0.85 0.35 0.10], [0.10 0.65 0.25]};

% Panel title
uicontrol('Style','text','Units','normalized',...
    'Position',[0.03 0.30 0.46 0.04],...
    'String','PID Gains  —  drag sliders to tune',...
    'FontSize',11,'FontWeight','bold','BackgroundColor','w',...
    'HorizontalAlignment','left');

for i = 1:3
    cfg = slider_cfg{i};
    y   = 0.22 - (i-1)*0.072;

    uicontrol('Style','text','Units','normalized',...
        'Position',[0.03 y+0.01 0.05 0.03],...
        'String',cfg{1},'FontSize',11,'FontWeight','bold',...
        'ForegroundColor',slider_colors{i},...
        'BackgroundColor','w','HorizontalAlignment','center');

    sliders(i) = uicontrol('Style','slider','Units','normalized',...
        'Position',[0.09 y+0.01 0.32 0.03],...
        'Min',cfg{2},'Max',cfg{3},'Value',cfg{4},...
        'BackgroundColor',[0.88 0.88 0.88]);

    val_labels(i) = uicontrol('Style','text','Units','normalized',...
        'Position',[0.42 y+0.01 0.07 0.03],...
        'String',sprintf('%.1f',cfg{4}),'FontSize',11,...
        'FontWeight','bold','ForegroundColor',slider_colors{i},...
        'BackgroundColor','w');

    addlistener(sliders(i),'Value','PostSet',@(~,~) update_all());
end

% Range labels
ranges = {'[0–200]','[0–100]','[0–20]'};
for i=1:3
    y = 0.22 - (i-1)*0.072;
    uicontrol('Style','text','Units','normalized',...
        'Position',[0.03 y-0.015 0.12 0.02],...
        'String',ranges{i},'FontSize',8,'BackgroundColor','w',...
        'ForegroundColor',[0.5 0.5 0.5]);
end

% -------------------------------------------------------------------------
%  Initial draw
% -------------------------------------------------------------------------
update_all();

% =========================================================================
%  UPDATE FUNCTION — called on every slider move
% =========================================================================
    function update_all()
        Kp = sliders(1).Value;
        Ki = sliders(2).Value;
        Kd = sliders(3).Value;

        val_labels(1).String = sprintf('%.1f', Kp);
        val_labels(2).String = sprintf('%.1f', Ki);
        val_labels(3).String = sprintf('%.1f', Kd);

        % ---- Build closed-loop system ----
        C_pid = Kp + Ki/s + Kd*s;
        C_p   = Kp;
        C_pd  = Kp + Kd*s;

        T_pid = feedback(C_pid*G, 1);
        T_p   = feedback(C_p*G,   1);
        T_pd  = feedback(C_pd*G,  1);

        % ---- Check stability ----
        poles_cl = pole(T_pid);
        is_stable = all(real(poles_cl) < 0);
        max_rp    = max(real(poles_cl));

        % ---- Step responses ----
        try
            [y_pid, ~] = step(T_pid, t_sim);
            [y_p,   ~] = step(T_p,   t_sim);
            [y_pd,  ~] = step(T_pd,  t_sim);
            valid = true;
        catch
            valid = false;
        end

        % ---- Compute metrics ----
        metrics = compute_metrics(y_pid, t_sim, is_stable);

        % ---- Update step response plot ----
        cla(ax_step); hold(ax_step,'on'); grid(ax_step,'on');
        yline(ax_step,1,'k--','LineWidth',1.5);

        if valid && is_stable
            plot(ax_step, t_sim, y_pid, '-','Color',[0.15 0.45 0.85],...
                'LineWidth',2.5,'DisplayName','PID response');

            % Annotate metrics on plot
            if ~isnan(metrics.rise_time)
                xline(ax_step, metrics.rise_time,'--','Color',[0.5 0.5 0.5],...
                    'LineWidth',1,'Label','t_r','FontSize',8);
            end
            if ~isnan(metrics.settling_time)
                xline(ax_step, metrics.settling_time,'--','Color',[0.7 0.4 0],...
                    'LineWidth',1,'Label','t_s','FontSize',8);
            end
            if ~isnan(metrics.overshoot) && metrics.overshoot > 0.5
                [pk,pk_idx] = max(y_pid);
                scatter(ax_step, t_sim(pk_idx), pk, 80,'r','filled',...
                    'DisplayName',sprintf('OS=%.1f%%',metrics.overshoot));
            end
        elseif valid
            % Unstable — plot briefly then clip
            y_clip = min(max(y_pid,-3),5);
            plot(ax_step, t_sim, y_clip, '-','Color',[0.9 0.2 0.2],...
                'LineWidth',2,'DisplayName','UNSTABLE');
            text(ax_step,t_sim(end)*0.4,2.5,'⚠  UNSTABLE',...
                'Color',[0.9 0.1 0.1],'FontSize',16,'FontWeight','bold',...
                'HorizontalAlignment','center');
        end
        xlabel(ax_step,'Time (s)','FontSize',10);
        ylabel(ax_step,'Joint Angle (rad)','FontSize',10);
        title(ax_step,sprintf('Step Response  |  Kp=%.1f  Ki=%.1f  Kd=%.1f',...
            Kp,Ki,Kd),'FontSize',10,'FontWeight','bold');

        % ---- Update pole-zero map ----
        cla(ax_pz); hold(ax_pz,'on'); grid(ax_pz,'on');
        xline(ax_pz,0,'k-','LineWidth',1.2);
        yline(ax_pz,0,'k-','LineWidth',1.2);

        % Shade unstable region
        xl = xlim(ax_pz);
        fill(ax_pz,[0 10 10 0],[-10 -10 10 10],[1 0.85 0.85],...
            'FaceAlpha',0.3,'EdgeColor','none','DisplayName','Unstable RHP');

        % Plant open-loop poles (×)
        p_ol = pole(G);
        scatter(ax_pz, real(p_ol), imag(p_ol), 80,'k','x',...
            'LineWidth',2,'SizeData',120,'DisplayName','OL poles');

        % Closed-loop poles
        for pi_idx = 1:length(poles_cl)
            pc = poles_cl(pi_idx);
            if real(pc) >= 0
                col = [0.9 0.1 0.1];
            else
                col = [0.15 0.45 0.85];
            end
            scatter(ax_pz, real(pc), imag(pc), 80, col, 'filled',...
                'MarkerEdgeColor','k','DisplayName','CL poles');
        end

        % Controller zeros
        z_c = zero(C_pid);
        if ~isempty(z_c)
            scatter(ax_pz, real(z_c), imag(z_c), 80,...
                [0.1 0.65 0.25],'o','LineWidth',2,'DisplayName','C zeros');
        end

        % Auto-scale around poles
        all_pts = [real(poles_cl); real(z_c); real(p_ol)];
        xlim(ax_pz, [min(all_pts)-3, max(abs(all_pts))+2]);
        legend(ax_pz,'Location','northeast','FontSize',7);
        xlabel(ax_pz,'Real'); ylabel(ax_pz,'Imag');
        title(ax_pz,'Pole-Zero Map','FontSize',11,'FontWeight','bold');

        % ---- Update build-up plot ----
        cla(ax_build); hold(ax_build,'on'); grid(ax_build,'on');
        yline(ax_build,1,'k--','LineWidth',1.2);
        if valid
            try
                plot(ax_build, t_sim, y_p,  '-','Color',[0.6 0.6 0.6],...
                    'LineWidth',1.5,'DisplayName',sprintf('P only (Kp=%.0f)',Kp));
                plot(ax_build, t_sim, y_pd, '--','Color',[0.85 0.5 0.1],...
                    'LineWidth',1.8,'DisplayName',sprintf('PD (Kd=%.1f)',Kd));
                plot(ax_build, t_sim, y_pid,'-','Color',[0.15 0.45 0.85],...
                    'LineWidth',2.2,'DisplayName',sprintf('PID (Ki=%.1f)',Ki));
            catch
            end
        end
        legend(ax_build,'Location','best','FontSize',7.5);
        xlabel(ax_build,'Time (s)','FontSize',9);
        title(ax_build,'P → PD → PID','FontSize',10,'FontWeight','bold');

        % ---- Update info panel ----
        info_h.String = build_info_string(Kp,Ki,Kd,metrics,is_stable,...
            poles_cl,max_rp,J,B);
    end

% =========================================================================
%  HELPER FUNCTIONS
% =========================================================================
    function m = compute_metrics(y, t, stable)
        m.rise_time    = NaN;
        m.overshoot    = NaN;
        m.settling_time= NaN;
        m.ss_error     = NaN;

        if ~stable || isempty(y) || any(isnan(y)) || any(isinf(y))
            return;
        end

        y_ss = y(end);
        m.ss_error = abs(1 - y_ss) * 100;   % % of setpoint

        % Rise time: 10% → 90% of final value
        i10 = find(y >= 0.10*y_ss, 1);
        i90 = find(y >= 0.90*y_ss, 1);
        if ~isempty(i10) && ~isempty(i90)
            m.rise_time = t(i90) - t(i10);
        end

        % Overshoot
        [y_max, ~] = max(y);
        if y_ss > 0.01
            m.overshoot = max(0, (y_max - y_ss)/y_ss * 100);
        end

        % Settling time (within ±2% of y_ss)
        band = 0.02 * abs(y_ss);
        settled = find(abs(y - y_ss) > band, 1, 'last');
        if ~isempty(settled) && settled < length(t)
            m.settling_time = t(settled);
        else
            m.settling_time = t(end);
        end
    end

    function str = build_info_string(Kp,Ki,Kd,m,stable,poles,max_rp,J,B)
        if stable
            stab_str = '✓ STABLE';
            stab_col_char = '';
        else
            stab_str = sprintf('✖ UNSTABLE  (max Re(p)=+%.3f)',max_rp);
        end

        str = sprintf([...
            ' Plant:  G(s) = 1/(Js²+Bs)\n',...
            '   J = %.2f kg·m²    B = %.2f N·m·s/rad\n\n',...
            ' PID Gains:\n',...
            '   Kp = %7.2f\n',...
            '   Ki = %7.2f\n',...
            '   Kd = %7.2f\n\n',...
            ' Stability: %s\n\n',...
            ' Performance Metrics:\n',...
            '   Rise time     = %s s\n',...
            '   Overshoot     = %s %%\n',...
            '   Settling time = %s s\n',...
            '   SS error      = %s %%\n\n',...
            ' Closed-Loop Poles:\n'],...
            J, B, Kp, Ki, Kd, stab_str,...
            fmt_val(m.rise_time),    fmt_val(m.overshoot),...
            fmt_val(m.settling_time),fmt_val(m.ss_error));

        for i = 1:length(poles)
            pc = poles(i);
            if imag(pc) >= 0
                if real(pc) >= 0
                    tag = ' ← ✖ RHP';
                else
                    tag = '';
                end
                str = [str, sprintf('   %+.3f %+.3fi%s\n',...
                    real(pc),imag(pc),tag)]; %#ok<AGROW>
            end
        end

        str = [str, sprintf(['\n Tuning Tips:\n',...
            '   Kp↑ → faster, more overshoot\n',...
            '   Kd↑ → damps overshoot\n',...
            '   Ki↑ → eliminates SS error\n',...
            '         (but can cause wind-up)'])];
    end

    function s = fmt_val(v)
        if isnan(v)
            s = '  N/A  ';
        else
            s = sprintf('%.4f', v);
        end
    end
