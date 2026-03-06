% =========================================================================
% INVERSE KINEMATICS - Analytical Solution for 2-DOF Planar Arm
% =========================================================================
% Given a desired end-effector position (x, y), find the joint angles
% θ1 and θ2 using the geometric/analytical approach.
%
% Two solutions exist (elbow-up / elbow-down):
%   cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)
%   θ2 = ±atan2(sin(θ2), cos(θ2))
%   θ1 = atan2(y, x) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))
%
% INPUTS (prompted):  target (x, y),  link lengths L1 and L2
% =========================================================================

fprintf('=== 2-DOF Planar Arm — Analytical Inverse Kinematics ===\n\n');

L1 = input('Enter link 1 length L1 (m) [default 1.0]: ');
if isempty(L1), L1 = 1.0; end
L2 = input('Enter link 2 length L2 (m) [default 0.8]: ');
if isempty(L2), L2 = 0.8; end

fprintf('\nReachable workspace radius: [%.2f, %.2f] m\n', abs(L1-L2), L1+L2);
target = input('Enter target position [x y]: ');
xd = target(1);  yd = target(2);

% --- Analytical IK ---
r2   = xd^2 + yd^2;
c2   = (r2 - L1^2 - L2^2) / (2*L1*L2);

if abs(c2) > 1
    error('Target (%.2f, %.2f) is outside the reachable workspace!', xd, yd);
end

s2_pos =  sqrt(1 - c2^2);   % elbow-down
s2_neg = -sqrt(1 - c2^2);   % elbow-up

solutions = struct();

for sol = 1:2
    if sol == 1
        s2 = s2_pos; label = 'Elbow-Down';
    else
        s2 = s2_neg; label = 'Elbow-Up';
    end
    th2 = atan2(s2, c2);
    th1 = atan2(yd, xd) - atan2(L2*s2, L1 + L2*c2);
    solutions(sol).th1   = th1;
    solutions(sol).th2   = th2;
    solutions(sol).label = label;
    fprintf('\n[%s]  θ1=%.2f°  θ2=%.2f°\n', label, rad2deg(th1), rad2deg(th2));
    % Verify
    x_fk = L1*cos(th1) + L2*cos(th1+th2);
    y_fk = L1*sin(th1) + L2*sin(th1+th2);
    fprintf('  FK check: (%.4f, %.4f)  target: (%.4f, %.4f)\n', x_fk, y_fk, xd, yd);
end

% --- Visualise both solutions ---
figure('Name','Inverse Kinematics - 2DOF Planar','Color','w');
cols = {'b','r'};
for sol = 1:2
    subplot(1,2,sol);
    th1 = solutions(sol).th1;
    th2 = solutions(sol).th2;

    % Joint positions
    J0 = [0; 0];
    J1 = J0 + L1*[cos(th1); sin(th1)];
    J2 = J1 + L2*[cos(th1+th2); sin(th1+th2)];

    hold on; grid on; axis equal;
    lim = L1+L2+0.3;
    axis([-lim lim -lim lim]);
    xlabel('X (m)'); ylabel('Y (m)');
    title(solutions(sol).label, 'FontSize', 12);

    % Workspace circle
    th_circ = linspace(0, 2*pi, 200);
    plot((L1+L2)*cos(th_circ), (L1+L2)*sin(th_circ), 'k--','LineWidth',0.5);
    plot(abs(L1-L2)*cos(th_circ), abs(L1-L2)*sin(th_circ), 'k:','LineWidth',0.5);

    % Arm
    pts = [J0, J1, J2];
    plot(pts(1,:), pts(2,:), [cols{sol},'-o'], 'LineWidth', 3, ...
        'MarkerSize', 8, 'MarkerFaceColor', cols{sol});

    % Target
    scatter(xd, yd, 120, 'g', 'filled', 'Marker','pentagram');
    text(xd+0.05, yd+0.05, sprintf('Target\n(%.2f,%.2f)',xd,yd), ...
        'Color','g','FontWeight','bold');
    text(J0(1)-0.15, J0(2)-0.15, 'J_0','FontWeight','bold');
    text(J1(1)+0.05, J1(2)+0.05, 'J_1','FontWeight','bold');
    text(J2(1)+0.05, J2(2)+0.05, 'EE','Color',cols{sol},'FontWeight','bold');

    % Angle arcs
    draw_angle_arc(J0, th1, 0.25, 'k');
    draw_angle_arc(J1, th1+th2, th1, 0.2, 'm');
end
sgtitle(sprintf('2-DOF IK  |  L1=%.1f  L2=%.1f  |  Target=(%.2f, %.2f)', ...
    L1, L2, xd, yd));

% -------------------------------------------------------------------------
function draw_angle_arc(center, angle_end, angle_start, r, color)
    if nargin == 4, color = angle_start; angle_start = 0; end
    th = linspace(angle_start, angle_end, 40);
    x  = center(1) + r*cos(th);
    y  = center(2) + r*sin(th);
    plot(x, y, color, 'LineWidth', 1.5);
end
