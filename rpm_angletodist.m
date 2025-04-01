clc;
clear all;
close all;

%% parameters
% radius of ball
ball_rad = 0.12;
% mass of ball 
ball_mass = 0.6;
% radius of flywheel
fly_rad = 0.075;
% mass of flywheel
fly_mass = 1.5;
% moment of inertia of flywheel
I_flywheel = 0.0057;
% moment of inertia of ball
I_ball = 0.00622;
% ramp up motor time
ramp_up_time = 2;
% height of basket from shooter
h = -1;
% gravitational constant
g = -9.81;

%% Calculating at certain rpm and angle what distance traveled by x
rpm = (0:1:4000)'; % RPM values
angle = (0:0.1:90); % Angle values
wheel_final_rot = (rpm*2*pi/60) ./ (1 + (ball_mass + I_ball/(ball_rad^2)) * ((fly_rad^2) / (4 * I_flywheel))); % Final rotational speed of the wheel
proj_vel = wheel_final_rot * fly_rad / 2; % Projectile velocity

% Initialize a matrix to store the results for t
t = NaN(length(proj_vel), length(angle));

% Loop over each combination of proj_vel and angle
for i = 1:length(proj_vel)
    for j = 1:length(angle)
        % Calculate the two possible values of t for each combination
        term1 = -(proj_vel(i) * sind(angle(j))); % Vertical component of velocity
        term2 = sqrt((proj_vel(i) * sind(angle(j)))^2 + 2 * g * h); % Discriminant part

        % Calculate the two roots of t
        t1 = (term1 + term2) / g; % First root
        t2 = (term1 - term2) / g; % Second root

        % Apply condition to keep only real and positive values
        if isreal(t1) && t1 > 0
            t(i, j) = t1; % Assign t1 if valid
        elseif isreal(t2) && t2 > 0
            t(i, j) = t2; % Assign t2 if valid
        end
    end
end

% Calculate the distance (x_dist) using the projectile velocity and time
x_dist = proj_vel * cosd(angle) .* t;

% Create the output matrix with RPM values as the first column
output_matrix = [rpm, x_dist]; % Add RPM as the first column

% Prepend the angle values as the first row (size [1, 901])
output_matrix = [0,angle; output_matrix]; % Add the angle row as the first row

% Write the output matrix to a CSV file
writematrix(output_matrix, "bruh.csv");

