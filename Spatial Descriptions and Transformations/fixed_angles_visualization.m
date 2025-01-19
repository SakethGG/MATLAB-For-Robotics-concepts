
% This code helps you visualize and understand fixed-angle rotations in 3D space.
% It demonstrates how a coordinate frame rotates step-by-step relative to a fixed frame, 
% based on the order of rotations (e.g., XYZ, ZYX) and angles you provide.
%
% Theory:
% In fixed-angle rotation, the frame rotates around the fixed (global) axes in sequence.
% The order of rotations (e.g., first X, then Y, then Z) determines the final orientation.
%
% Rotation Matrix Multiplication:
% - To combine rotations, multiply the rotation matrices in the reverse order of the specified sequence.
% - For example, if the rotation order is ZYX, compute the combined matrix as R = Rx * Ry * Rz.

%
% Key features:
% - Demonstrates step-by-step rotations using the fixed-angle convention.
% - Animates the rotation process for easy visualization.
% - Highlights how rotation order affects the result.


% Define the fixed-angle rotation sequence and initial angles
rotation_order = input('Enter the rotation order (e.g., ZYX): ', 's');
alpha = input('Enter the first rotation angle (in degrees): ');
beta = input('Enter the second rotation angle (in degrees): ');
gamma = input('Enter the third rotation angle (in degrees): ');

% Convert angles to radians
alpha = deg2rad(alpha);
beta = deg2rad(beta);
gamma = deg2rad(gamma);

% Define rotation matrices for fixed axes
R_x = @(theta) [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
R_y = @(theta) [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
R_z = @(theta) [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];

% Initialize the total rotation matrix
R_total = eye(3);

% Plot initial coordinate frame
figure;
axis([-2 2 -2 2 -2 2]);
grid on;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% Plot the fixed coordinate frame
plot_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za'); % Fixed frame in blue

% Initial plot for rotating frame
plot_axes(R_total, 'k', 'Initial', 'Xb', 'Yb', 'Zb');

% Rotation sequence for fixed angles
for i = 1:length(rotation_order)
    switch rotation_order(i)
        case 'X'
            angles = linspace(0, alpha, 100); % Fixed-angle rotation around X
            for angle = angles
                R_step = R_x(angle); % Compute incremental rotation
                R_current = R_step * R_total; % Apply rotation to the total
                % Clear previous plots
                cla;
                hold on;
                % Plot fixed coordinate frame
                plot_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za');
                % Plot updated rotating coordinate frame
                plot_axes(R_current, 'r', 'Rotated', 'Xb', 'Yb', 'Zb');
                pause(0.01); % Smooth animation
            end
            R_total = R_x(alpha) * R_total; % Update total rotation matrix
        case 'Y'
            angles = linspace(0, beta, 100); % Fixed-angle rotation around Y
            for angle = angles
                R_step = R_y(angle);
                R_current = R_step * R_total;
                % Clear previous plots
                cla;
                hold on;
                % Plot fixed coordinate frame
                plot_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za');
                % Plot updated rotating coordinate frame
                plot_axes(R_current, 'g', 'Rotated', 'Xb', 'Yb', 'Zb');
                pause(0.01);
            end
            R_total = R_y(beta) * R_total;
        case 'Z'
            angles = linspace(0, gamma, 100); % Fixed-angle rotation around Z
            for angle = angles
                R_step = R_z(angle);
                R_current = R_step * R_total;
                % Clear previous plots
                cla;
                hold on;
                % Plot fixed coordinate frame
                plot_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za');
                % Plot updated rotating coordinate frame
                plot_axes(R_current, 'm', 'Rotated', 'Xb', 'Yb', 'Zb');
                pause(0.01);
            end
            R_total = R_z(gamma) * R_total;
    end
end

hold off;

% Function to plot coordinate axes with arrowheads and labels
function plot_axes(R, color, label, X_label, Y_label, Z_label)
    % Define the original axes
    orig_axes = [1 0 0; 0 1 0; 0 0 1];
    % Apply rotation
    rotated_axes = (R * orig_axes')';
    % Arrowhead length
    arrow_length = 1.0;
    
    % Plot X-axis
    quiver3(0, 0, 0, rotated_axes(1,1), rotated_axes(1,2), rotated_axes(1,3), ...
        'Color', color, 'LineWidth', 2, 'MaxHeadSize', 1.5, 'DisplayName', [label, ' X']);
    % Plot Y-axis
    quiver3(0, 0, 0, rotated_axes(2,1), rotated_axes(2,2), rotated_axes(2,3), ...
        'Color', color, 'LineWidth', 2, 'MaxHeadSize', 1.5, 'DisplayName', [label, ' Y']);
    % Plot Z-axis
    quiver3(0, 0, 0, rotated_axes(3,1), rotated_axes(3,2), rotated_axes(3,3), ...
        'Color', color, 'LineWidth', 2, 'MaxHeadSize', 1.5, 'DisplayName', [label, ' Z']);
    
    % Label axes
    text(rotated_axes(1,1) * 1.1, rotated_axes(1,2) * 1.1, rotated_axes(1,3) * 1.1, ...
        X_label, 'Color', color, 'FontWeight', 'bold');
    text(rotated_axes(2,1) * 1.1, rotated_axes(2,2) * 1.1, rotated_axes(2,3) * 1.1, ...
        Y_label, 'Color', color, 'FontWeight', 'bold');
    text(rotated_axes(3,1) * 1.1, rotated_axes(3,2) * 1.1, rotated_axes(3,3) * 1.1, ...
        Z_label, 'Color', color, 'FontWeight', 'bold');
end
