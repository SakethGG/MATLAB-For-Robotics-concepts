% =========================================================================
% JACOBIAN MATRIX - Computation, Singularities & Manipulability
% =========================================================================
% The Jacobian J maps joint velocities to end-effector velocities:
%   ẋ = J(θ) · θ̇
%
% Key concepts visualised:
%   1. Analytical Jacobian for a 3-DOF planar arm
%   2. Manipulability ellipse  w = sqrt(det(J·Jᵀ))
%   3. Singular configurations (det(J·Jᵀ) ≈ 0)
%   4. Condition number κ(J) as a measure of isotropy
%
% No inputs needed — interactive slider-based exploration.
% =========================================================================

fprintf('=== Jacobian & Manipulability Explorer ===\n');
fprintf('Use sliders to change joint angles and observe the Jacobian live.\n\n');

L = [1.0, 0.8, 0.5];   % link lengths
n = length(L);

% --- Build interactive figure ---
fig = figure('Name','Jacobian & Manipulability','Color','w', ...
    'Position',[100 100 1100 550]);

% Arm axes
ax_arm = subplot(1,2,1);
hold(ax_arm,'on'); grid(ax_arm,'on'); axis(ax_arm,'equal');
lim = sum(L)+0.3;
axis(ax_arm,[-lim lim -lim lim]);
xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
title(ax_arm,'Robot Arm + Manipulability Ellipse');

% Info axes
ax_info = subplot(1,2,2);
axis(ax_info,'off');

% Sliders
theta0 = [30, -45, 20];   % initial joint angles in degrees
sliders = gobjects(n,1);
for i = 1:n
    sliders(i) = uicontrol('Style','slider','Min',-180,'Max',180,...
        'Value',theta0(i),'Units','normalized',...
        'Position',[0.05, 0.08-(i-1)*0.06, 0.3, 0.04]);
    uicontrol('Style','text','Units','normalized',...
        'Position',[0.01, 0.085-(i-1)*0.06, 0.04, 0.03],...
        'String',sprintf('θ%d',i),'FontWeight','bold');
    addlistener(sliders(i),'Value','PostSet',@(~,~) update_plot());
end

update_plot();   % initial draw

    function update_plot()
        theta = deg2rad([sliders(1).Value, sliders(2).Value, sliders(3).Value]);

        % FK
        [joints, ee] = fk_planar(theta, L);
        J = planar_jacobian(theta, L);

        % Manipulability
        JJt  = J * J';
        w    = sqrt(max(det(JJt), 0));
        cond_num = cond(J);

        % --- Draw arm ---
        cla(ax_arm); hold(ax_arm,'on'); grid(ax_arm,'on');
        axis(ax_arm,'equal');
        axis(ax_arm,[-lim lim -lim lim]);
        pts = joints;
        plot(ax_arm, pts(:,1), pts(:,2), 'b-o','LineWidth',3,'MarkerSize',8,'MarkerFaceColor','b');

        % --- Manipulability ellipse ---
        if w > 1e-4
            [V, D] = eig(JJt);
            semi_axes = sqrt(diag(D));
            th_ell  = linspace(0, 2*pi, 100);
            ellipse  = V * diag(semi_axes) * [cos(th_ell); sin(th_ell)];
            scale    = 0.4 / max(semi_axes);
            fill(ax_arm, ee(1)+scale*ellipse(1,:), ee(2)+scale*ellipse(2,:), ...
                [1 0.7 0.7], 'FaceAlpha',0.5,'EdgeColor','r','LineWidth',1.5);
            % velocity arrows along principal axes
            for k = 1:2
                v = scale * semi_axes(k) * V(:,k);
                quiver(ax_arm, ee(1),ee(2), v(1),v(2),'r','LineWidth',2,'MaxHeadSize',3,'AutoScale','off');
            end
        end

        scatter(ax_arm, ee(1),ee(2), 80,'r','filled');
        xlabel(ax_arm,'X (m)'); ylabel(ax_arm,'Y (m)');
        title(ax_arm, sprintf('Manipulability w=%.4f  |  κ(J)=%.2f', w, cond_num));
        if w < 0.05
            text(ax_arm, -lim+0.1, -lim+0.2, '⚠ Near Singularity!','Color','r','FontWeight','bold','FontSize',12);
        end

        % --- Display Jacobian ---
        cla(ax_info); axis(ax_info,'off');
        str = sprintf('Joint Angles:\n  θ1=%.1f°  θ2=%.1f°  θ3=%.1f°\n\n', ...
            rad2deg(theta(1)), rad2deg(theta(2)), rad2deg(theta(3)));
        str = [str, sprintf('Jacobian J (2×3):\n')];
        for row = 1:2
            str = [str, sprintf('  [%.3f  %.3f  %.3f]\n', J(row,1),J(row,2),J(row,3))];
        end
        str = [str, sprintf('\nManipulability  w = %.4f\n', w)];
        str = [str, sprintf('Condition No.   κ = %.2f\n', cond_num)];
        str = [str, sprintf('\nSingular if w → 0\nIsotropic if κ → 1')];
        text(ax_info, 0.05, 0.95, str,'Units','normalized','VerticalAlignment','top',...
            'FontName','Courier','FontSize',10,'Interpreter','none');
    end

% =========================================================================
function [joints, ee] = fk_planar(theta, L)
    n = length(L);
    joints = zeros(n+1, 2);
    cum_th = 0;
    for i = 1:n
        cum_th = cum_th + theta(i);
        joints(i+1,:) = joints(i,:) + L(i)*[cos(cum_th), sin(cum_th)];
    end
    ee = joints(end,:);
end

function J = planar_jacobian(theta, L)
    n = length(L);
    J = zeros(2, n);
    [joints, ee] = fk_planar(theta, L);
    for i = 1:n
        r = ee - joints(i,:);
        J(:,i) = [-r(2); r(1)];
    end
end
