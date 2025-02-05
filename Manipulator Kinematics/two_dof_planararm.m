clc; clear; close all;

% Link lengths
L1 = 1;  % Length of first link
L2 = 0.8;  % Length of second link

% User inputs joint angles (in degrees)
theta1 = input('Enter theta1 (in degrees): ');
theta2 = input('Enter theta2 (in degrees): ');

% Convert angles to radians
theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);

% Forward Kinematics
x0 = 0; z0 = 0; % Base position
x1 = L1 * cos(theta1);
z1 = L1 * sin(theta1);
x2 = x1 + L2 * cos(theta1 + theta2);
z2 = z1 + L2 * sin(theta1 + theta2);

% Animation setup
figure; axis equal; hold on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('3D 2-DOF Planar Manipulator Animation in X-Z Plane');
grid on;
view(3);

% Initial position (dotted line)
plot3([x0, L1], [0, 0], [z0, 0], 'r--', 'LineWidth', 1.5);
plot3([L1, L1 + L2], [0, 0], [0, 0], 'b--', 'LineWidth', 1.5);

% Animate first link rotation
steps = 50;
theta1_vals = linspace(0, theta1, steps);
for i = 1:steps
    x1_temp = L1 * cos(theta1_vals(i));
    z1_temp = L1 * sin(theta1_vals(i));
    
    cla;
    plot3([x0, L1], [0, 0], [z0, 0], 'r--', 'LineWidth', 1.5);
    plot3([L1, L1 + L2], [0, 0], [0, 0], 'b--', 'LineWidth', 1.5);
    plot3([x0, x1_temp], [0, 0], [z0, z1_temp], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
    plot3(x1_temp, 0, z1_temp, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    view(3);
    pause(0.05);
end

% Animate second link rotation after first link finishes
theta2_vals = linspace(0, theta2, steps);
for i = 1:steps
    % Link 1 has fully rotated, now rotate link 2
    x2_temp = x1 + L2 * cos(theta1 + theta2_vals(i));
    z2_temp = z1 + L2 * sin(theta1 + theta2_vals(i));
    
    cla;
    plot3([x0, L1], [0, 0], [z0, 0], 'r--', 'LineWidth', 1.5);
    plot3([L1, L1 + L2], [0, 0], [0, 0], 'b--', 'LineWidth', 1.5);
    plot3([x0, x1], [0, 0], [z0, z1], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
    plot3([x1, x2_temp], [0, 0], [z1, z2_temp], 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
    plot3(x2_temp, 0, z2_temp, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    view(3);
    pause(0.05);
end

% Final static plot
plot3([x0, x1], [0, 0], [z0, z1], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
plot3([x1, x2], [0, 0], [z1, z2], 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
plot3(x2, 0, z2, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
text(x0, 0, z0, 'Base', 'FontSize', 12, 'Color', 'k');
text(x1, 0, z1, 'Joint 1', 'FontSize', 12, 'Color', 'r');
text(x2, 0, z2, 'End-Effector', 'FontSize', 12, 'Color', 'g');
view(3);
