% This code helps you visualize and understand the equivalent angle-axis representation of rotations in 3D space.
% It demonstrates how a coordinate frame rotates step-by-step relative to a fixed frame, 
% based on a specified rotation axis (unit vector) and rotation angle (in degrees).
%
% Theory:
% The equivalent angle-axis representation describes a rotation as a rotation around a fixed axis by a specified angle.
% - The rotation axis is given as a unit vector (direction in space).
% - The angle of rotation is applied around this axis.
%
% Rotation Matrix for Equivalent Angle-Axis Representation:
% The rotation matrix R for an angle-axis rotation is derived using Rodrigues' rotation formula:
% R = I + sin(theta) * K + (1 - cos(theta)) * K^2
% where:
% - I is the identity matrix
% - K is the skew-symmetric matrix of the rotation axis vector
% - theta is the angle of rotation (in radians).
%
% Key features:
% - Demonstrates rotations around a specified axis by an angle using the axis-angle convention.
% - Animates the rotation process for easy visualization.
% - Highlights how changing the axis and angle affects the final orientation.


% Define rotation axis (unit vector) and angle
axis_vector = input('Enter the rotation axis (x, y, z) as a vector [x, y, z]: ');  % Renamed variable
angle = input('Enter the rotation angle (in degrees): ');

% Normalize the axis vector to ensure it's a unit vector
axis = axis_vector / norm(axis_vector);

% Convert the angle from degrees to radians
theta = deg2rad(angle);

% Define the skew-symmetric matrix K for the axis of rotation
K = [ 0, -axis(3), axis(2);
      axis(3), 0, -axis(1);
     -axis(2), axis(1), 0 ];

% Define the identity matrix
I = eye(3);

% Compute the rotation matrix using Rodrigues' rotation formula
R = I + sin(theta) * K + (1 - cos(theta)) * (K^2);

% Plot initial coordinate frame
figure;
set_axis_limits([-2 2 -2 2 -2 2]);  % Using a new function for axis limits
grid on;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% Plot the fixed coordinate frame (stationary)
plot_coordinate_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za');
% Fixed frame in blue

% Initial plot for rotating frame
plot_coordinate_axes(R, 'r', 'Rotated', 'Xb', 'Yb', 'Zb');

% Smooth animation by interpolating the rotation
num_steps = 100;
angles = linspace(0, theta, num_steps);
for angle_step = angles
    % Compute the incremental rotation matrix
    R_step = I + sin(angle_step) * K + (1 - cos(angle_step)) * (K^2);
    
    % Clear previous plots
    cla;
    hold on;
    
    % Plot fixed coordinate frame
    plot_coordinate_axes(eye(3), 'b', 'Fixed', 'Xa', 'Ya', 'Za'); % Fixed frame in blue
    
    % Plot updated rotating coordinate frame
    plot_coordinate_axes(R_step, 'r', 'Rotated', 'Xb', 'Yb', 'Zb');
    
    % Short pause to create smooth animation
    pause(0.01);
end

hold off;

% Function to set axis limits
function set_axis_limits(axis_limits)
    axis(axis_limits); % Set axis limits using the custom function
end

% Function to plot coordinate axes with arrowheads and labels
function plot_coordinate_axes(R, color, label, X_label, Y_label, Z_label)
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
