% % Clearing workspace, closing figures, and clearing command window
% close all
% clear
% clc
% 
% % Parameters of the vehicle
% r = 3.25;  % Wheel radius
% l = 8;     % Distance between wheel and center of the vehicle
% 
% % Initial position and orientation of the vehicle
% x0 = 0;
% y0 = 0;
% theta0 = pi/2;  % Start facing upward (90 degrees)
% 
% % Time span for each segment of motion (5 seconds per side/turn)
% tspan_straight = [0 5];  % Time to move straight
% tspan_straight_2 = [0 2];  % Time to move straight
% tspan_turn = [0 2];      % Time to turn 90 degrees
% 
% % Defining symbolic variables for solving ODEs
% syms x(t) y(t) theta(t)
% 
% % Function to move straight
% move_straight = @(x0, y0, theta0, tspan) ...
%     solve_trajectory(x0, y0, theta0, r, l, 5, 5, tspan);  % equal wheel velocities
% 
% % Function to turn 90 degrees (left turn)
% turn_90_deg = @(x0, y0, theta0, tspan) ...
%     solve_trajectory(x0, y0, theta0, r, l, 5, 1.143, tspan);  % right wheel moves, left wheel stops
% 
% % Initialize variables for trajectory
% X = [];
% Y = [];
% 
% % 1. First side (move straight)
% [xSol, ySol, thetaSol] = move_straight(x0, y0, theta0, tspan_straight);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 2. First 90-degree turn (left)
% [xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), thetaSol(end), tspan_turn);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 3. Second side (move straight)
% [xSol, ySol, thetaSol] = move_straight(X(end), Y(end), thetaSol(end), tspan_straight_2);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 4. Second 90-degree turn (left)
% [xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), thetaSol(end), tspan_turn);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 5. Third side (move straight)
% [xSol, ySol, thetaSol] = move_straight(X(end), Y(end), thetaSol(end), tspan_straight);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 6. Third 90-degree turn (left)
% [xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), thetaSol(end), tspan_turn);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 7. Fourth side (move straight)
% [xSol, ySol, thetaSol] = move_straight(X(end), Y(end), thetaSol(end), tspan_straight_2);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % 8. Fourth 90-degree turn (left to close the square)
% [xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), thetaSol(end), tspan_turn);
% X = [X; xSol];
% Y = [Y; ySol];
% 
% % Plotting the XY trajectory
% figure
% plot(X, Y, '-o', 'linewidth', 2), grid on
% title('XY Trajectory of Vehicle in a Square Path')
% xlabel('X Position (units)')
% ylabel('Y Position (units)')
% axis equal
% 
% % Function to solve the vehicle trajectory for a given wheel velocity
% function [xSol, ySol, thetaSol] = solve_trajectory(x0, y0, theta0, r, l, fhiRDot, fhiLDot, tspan)
% 
%     % Translational and rotational velocities
%     xrdot = (r / 2) * (fhiRDot + fhiLDot);     % Translational velocity
%     omegadot = (r / (2 * l)) * (fhiRDot - fhiLDot);  % Rotational velocity
%     
%     % Define symbolic variables
%     syms x(t) y(t) theta(t)
%     
%     % Kinematic differential equations
%     ode1 = diff(x) == cos(theta) * xrdot;
%     ode2 = diff(y) == sin(theta) * xrdot;
%     ode3 = diff(theta) == omegadot;
%     odes = [ode1; ode2; ode3];
%     
%     % Initial conditions
%     cond1 = x(0) == x0;
%     cond2 = y(0) == y0;
%     cond3 = theta(0) == theta0;
%     conds = [cond1; cond2; cond3];
%     
%     % Solving the ODEs
%     S = dsolve(odes, conds);
%     
%     % Extracting the solutions
%     xSol = double(subs(S.x, t, linspace(tspan(1), tspan(2), 100)))';
%     ySol = double(subs(S.y, t, linspace(tspan(1), tspan(2), 100)))';
%     thetaSol = double(subs(S.theta, t, linspace(tspan(1), tspan(2), 100)))';
% end

% Clearing workspace, closing figures, and clearing command window
close all
clear
clc

% Parameters of the vehicle
r = 3.25;  % Wheel radius
l = 8;     % Distance between wheel and center of the vehicle

% Initial position and orientation of the vehicle
x0 = 0;
y0 = 0;
theta0 = pi/2;  % Start facing upward (90 degrees)

% Time span for each segment of motion
tspan_straight = [0 5];  % Time to move straight (5 seconds per side)
tspan_straight_2 = [0 5];  % Time to move straight (2 seconds per side)
tspan_turn = [0 2];      % Time to turn 90 degrees (2 seconds per turn)

% Initialize variables for trajectory
X = [];
Y = [];
theta_traj = [];

% Function to move straight
move_straight = @(x0, y0, theta0, tspan) ...
    solve_trajectory(x0, y0, theta0, r, l, 10, 10, tspan);  % equal wheel velocities

% Function to turn 90 degrees (left turn)
turn_90_deg = @(x0, y0, theta0, tspan) ...
    solve_trajectory(x0, y0, theta0, r, l, 5, 1.143, tspan);  % right wheel moves, left wheel stops

% 1. First side (move straight)
[xSol, ySol, thetaSol] = move_straight(x0, y0, theta0, tspan_straight);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 2. First 90-degree turn (left)
[xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), theta_traj(end), tspan_turn);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 3. Second side (move straight)
[xSol, ySol, thetaSol] = move_straight(X(end), Y(end), theta_traj(end), tspan_straight_2);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 4. Second 90-degree turn (left)
[xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), theta_traj(end), tspan_turn);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 5. Third side (move straight)
[xSol, ySol, thetaSol] = move_straight(X(end), Y(end), theta_traj(end), tspan_straight);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 6. Third 90-degree turn (left)
[xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), theta_traj(end), tspan_turn);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 7. Fourth side (move straight)
[xSol, ySol, thetaSol] = move_straight(X(end), Y(end), theta_traj(end), tspan_straight_2);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% 8. Fourth 90-degree turn (left to close the square)
[xSol, ySol, thetaSol] = turn_90_deg(X(end), Y(end), theta_traj(end), tspan_turn);
X = [X; xSol];
Y = [Y; ySol];
theta_traj = [theta_traj; thetaSol];

% Animation setup
figure
hold on
grid on
title('XY Trajectory of Vehicle in a Square Path')
xlabel('X Position (units)')
ylabel('Y Position (units)')
axis equal
plot([-10, 10], [0, 0], 'k--')  % Static lines for reference
plot([0, 0], [-10, 10], 'k--')  % Static lines for reference

% Plot trajectory and animate vehicle movement
carBody = plot(0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
trajectoryLine = animatedline('LineWidth', 2, 'Color', 'r');

% Animation loop for each trajectory segment
nPoints = length(X);
for i = 1:nPoints
    % Update car position on plot
    set(carBody, 'XData', X(i), 'YData', Y(i));
    
    % Add points to the trajectory line
    addpoints(trajectoryLine, X(i), Y(i));
    
    % Pause to create animation effect
    pause(0.05);  % Adjust this value to control the speed of the animation
end

% Final plot to show complete trajectory
plot(X, Y, 'r-', 'LineWidth', 2);
hold off

% Function to solve the vehicle trajectory for a given wheel velocity
function [xSol, ySol, thetaSol] = solve_trajectory(x0, y0, theta0, r, l, fhiRDot, fhiLDot, tspan)

    % Translational and rotational velocities
    xrdot = (r / 2) * (fhiRDot + fhiLDot);     % Translational velocity
    omegadot = (r / (2 * l)) * (fhiRDot - fhiLDot);  % Rotational velocity
    
    % Define symbolic variables
    syms x(t) y(t) theta(t)
    
    % Kinematic differential equations
    ode1 = diff(x) == cos(theta) * xrdot;
    ode2 = diff(y) == sin(theta) * xrdot;
    ode3 = diff(theta) == omegadot;
    odes = [ode1; ode2; ode3];
    
    % Initial conditions
    cond1 = x(0) == x0;
    cond2 = y(0) == y0;
    cond3 = theta(0) == theta0;
    conds = [cond1; cond2; cond3];
    
    % Solving the ODEs
    S = dsolve(odes, conds);
    
    % Extracting the solutions
    xSol = double(subs(S.x, t, linspace(tspan(1), tspan(2), 100)))';
    ySol = double(subs(S.y, t, linspace(tspan(1), tspan(2), 100)))';
    thetaSol = double(subs(S.theta, t, linspace(tspan(1), tspan(2), 100)))';
end

