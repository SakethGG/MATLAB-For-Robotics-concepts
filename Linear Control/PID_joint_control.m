% =========================================================================
% LINEAR CONTROL - PD / PID Joint-Space Control
% =========================================================================
% Simulates joint-space PD and PID control of a 2-DOF arm under gravity.
%
% Control law:
%   τ = Kp·(qd - q) - Kd·q̇ + Ki·∫(qd - q)dt    [PID]
%   τ = Kp·(qd - q) - Kd·q̇                       [PD only]
%
% The robot dynamics (simplified with gravity):
%   q̈ = M(q)⁻¹ · [τ - C(q,q̇)·q̇ - g(q)]
%
% Students can tune Kp, Kd, Ki and observe:
%   - Rise time, overshoot, steady-state error
%   - Effect of gravity compensation
% =========================================================================

fprintf('=== Joint-Space PD/PID Control ===\n\n');
fprintf('Controller type:\n  1 - PD\n  2 - PID\n  3 - PD + Gravity Compensation\n');
ctrl_type = input('Choose [1/2/3, default 3]: ');
if isempty(ctrl_type), ctrl_type = 3; end

% --- Gains (students can tune these) ---
Kp = diag([50, 40]);
Kd = diag([10, 8]);
Ki = diag([5,  4]);

fprintf('\nDefault gains:  Kp=[50,40]  Kd=[10,8]  Ki=[5,4]\n');
ans_change = input('Change gains? (y/n) [n]: ', 's');
if strcmpi(ans_change,'y')
    kp_in = input('Enter Kp diagonal [kp1 kp2]: ');
    kd_in = input('Enter Kd diagonal [kd1 kd2]: ');
    ki_in = input('Enter Ki diagonal [ki1 ki2]: ');
    Kp = diag(kp_in);  Kd = diag(kd_in);  Ki = diag(ki_in);
end

% --- Robot parameters ---
m  = [1.0, 0.8];    % link masses (kg)
L  = [1.0, 0.8];    % link lengths (m)
lc = L/2;           % CoM at link midpoint
g  = 9.81;

% --- Simulation ---
T  = 5.0;  dt = 0.005;
t  = 0:dt:T;  N = length(t);

q_des = deg2rad([45, -60]);     % desired joint angles
q  = zeros(2, N);   q(:,1)  = deg2rad([0, 0]);
qd = zeros(2, N);
e_int = zeros(2, 1);
tau_hist = zeros(2, N);

for k = 1:N-1
    q_k  = q(:,k);
    qd_k = qd(:,k);
    e    = q_des(:) - q_k;
    e_int = e_int + e * dt;

    % --- Dynamics ---
    M  = inertia_matrix(q_k, m, L, lc);
    Cqd = coriolis_term(q_k, qd_k, m, L, lc);
    grav = gravity_term(q_k, m, L, lc, g);

    % --- Controller ---
    tau = Kp*e - Kd*qd_k;
    if ctrl_type == 2,  tau = tau + Ki*e_int;  end
    if ctrl_type == 3,  tau = tau + grav;       end   % gravity comp
    tau_hist(:,k) = tau;

    % --- Euler integration ---
    qdd = M \ (tau - Cqd - grav);
    qd(:,k+1) = qd_k + qdd * dt;
    q(:,k+1)  = q_k  + qd(:,k+1)*dt;
end

ctrl_names = {'PD','PID','PD + Gravity Compensation'};

% --- Plot results ---
figure('Name',['Control: ', ctrl_names{ctrl_type}],'Color','w','Position',[100 100 1100 500]);

subplot(1,3,1);
plot(t, rad2deg(q(1,:)), 'b', 'LineWidth', 2); hold on;
plot(t, rad2deg(q(2,:)), 'r', 'LineWidth', 2);
yline(rad2deg(q_des(1)), 'b--', 'LineWidth', 1.5);
yline(rad2deg(q_des(2)), 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('Angle (°)');
title('Joint Angles'); legend('θ1','θ2','θ1_d','θ2_d');
grid on;

subplot(1,3,2);
e1 = rad2deg(q_des(1)) - rad2deg(q(1,:));
e2 = rad2deg(q_des(2)) - rad2deg(q(2,:));
plot(t, e1, 'b', t, e2, 'r', 'LineWidth', 2);
xlabel('t (s)'); ylabel('Error (°)');
title('Tracking Error'); legend('e1','e2'); grid on;

subplot(1,3,3);
plot(t, tau_hist(1,:), 'b', t, tau_hist(2,:), 'r', 'LineWidth', 2);
xlabel('t (s)'); ylabel('Torque (Nm)');
title('Control Torques'); legend('τ1','τ2'); grid on;

sgtitle(sprintf('%s Control  |  Kp=[%.0f,%.0f]  Kd=[%.0f,%.0f]', ...
    ctrl_names{ctrl_type}, Kp(1,1),Kp(2,2),Kd(1,1),Kd(2,2)));

ss_error = rad2deg(abs(q_des(:) - q(:,end)));
fprintf('\nSteady-State Errors:  θ1=%.3f°  θ2=%.3f°\n', ss_error(1), ss_error(2));

% =========================================================================
function M = inertia_matrix(q, m, L, lc)
    m1=m(1);m2=m(2);L1=L(1);lc1=lc(1);lc2=lc(2);
    I1=m1*lc1^2; I2=m2*lc2^2;
    h  = m2*L1*lc2*cos(q(2));
    M11 = I1 + I2 + m2*L1^2 + 2*h;
    M12 = I2 + h;
    M  = [M11, M12; M12, I2];
end

function Cqd = coriolis_term(q, qd, m, L, lc)
    m2=m(2); L1=L(1); lc2=lc(2);
    h = m2*L1*lc2*sin(q(2));
    C = [-h*qd(2), -h*(qd(1)+qd(2)); h*qd(1), 0];
    Cqd = C * qd;
end

function gv = gravity_term(q, m, L, lc, g)
    m1=m(1);m2=m(2);L1=L(1);lc1=lc(1);lc2=lc(2);
    g1 = (m1*lc1 + m2*L1)*g*cos(q(1)) + m2*lc2*g*cos(q(1)+q(2));
    g2 = m2*lc2*g*cos(q(1)+q(2));
    gv = [g1; g2];
end
