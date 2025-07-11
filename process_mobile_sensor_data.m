%% TASK 1

% Load the sensor data file
data = load("C:\Users\Hamza\Downloads\sensorlog_last.mat"); 

% Extract variables
acceleration = data.Acceleration;
angular_velocity = data.AngularVelocity;

% Exclude the first column (time), and keep only x, y, z
acc_xyz = table2array(acceleration(:, 1:3));
ang_vel_xyz = table2array(angular_velocity(:, 1:3));

% Create time vector from 0 to 20 seconds
total_time = 20; 

% Plot angular velocity components
figure;
subplot(2,1,1);
time_gyro = linspace(0, total_time, 207);
plot(time_gyro, ang_vel_xyz);
title('Angular Velocity (X, Y, Z)');
xlabel('Time (s)');
ylabel('Angular Velocity');
legend('X', 'Y', 'Z');
grid on;

% Plot acceleration components
subplot(2,1,2);
time_acc = linspace(0, total_time, 206);
plot(time_acc, acc_xyz)
title('Acceleration (X, Y, Z)');
xlabel('Time (s)');
ylabel('Acceleration');
legend('X', 'Y', 'Z');
grid on;

%% TASK 2

% Sperate the measured accelerometer data 
acc_x = acc_xyz(:, 1);
acc_y = acc_xyz(:, 2);
acc_z = acc_xyz(:, 3);

% Construct a list to store the value of Roll and Pitch angles (in degrees)

num_samples = 206 ;
for i = 1:num_samples
    ax = acc_x(i);
    ay = acc_y(i);
    az = acc_z(i);
    
    % Compute roll and pitch for each sample
    roll = atan2(ay, sqrt(ax^2 + az^2));
    pitch = atan2(-ax, sqrt(ay^2 + az^2));

    %fprintf ('Sample %d -> Roll: %.2f deg, Pitch: %.2f deg\n', ...
            %i, rad2deg(roll), rad2deg(pitch));
end
%% TASK 3

for i = 1:206
    p = ang_vel_xyz(i, 1);  
    q = ang_vel_xyz(i, 2);  
    r = ang_vel_xyz(i, 3);  

    %fprintf('Sample %d -> p (X): %.3f, q (Y): %.3f, r (Z): %.3f\n', ...
            %i, p, q, r);
end

% Allocate roll and pitch arrays
roll = zeros(206, 1);
pitch = zeros(206, 1);

% Compute roll and pitch in radians
for i = 1:206
    ax = acc_x(i);
    ay = acc_y(i);
    az = acc_z(i);

    roll(i) = atan2(ay, sqrt(ax^2 + az^2));         
    pitch(i) = atan2(-ax, sqrt(ay^2 + az^2));   
   
end

% Extract gyro components into vectors
p = ang_vel_xyz(:, 1); 
q = ang_vel_xyz(:, 2);  
r = ang_vel_xyz(:, 3);  

% Get delta_t between samples
time_vector = table2array(data.Acceleration(:, 1));
delta_t = abs(diff(time_vector));  % 205 values

% Compute roll_dot and roll_new

roll_new = zeros(205, 1); % Make a new vector to store new values

for i = 1:205
    roll_dot = p(i) + sin(roll(i)) * tan(pitch(i)) * q(i) + cos(roll(i)) * tan(pitch(i)) * r(i);
    roll_new(i) = roll(i) + delta_t(i) * roll_dot;
    
    fprintf('Sample %d -> Roll_new: %.2f deg\n', i, rad2deg(roll_new(i)));
end


% Compute pitch_dot and pitch_new

pitch_new = zeros(205,1);

for i=1:205
    pitch_dot = q(i)*cos(roll(i)) - r(i)*sin(roll(i));
    pitch_new(i) = pitch(i) + delta_t(i) * pitch_dot;

    fprintf('Sample %d -> Pitch_new: %.2f deg\n', i, rad2deg(pitch_new(i)));
end
%% TASK 4 

% Assuming Constant Values for (A , u , C , R , Q) matrixes

A = 1; u = 5; C = 1 ; R = 1 ; Q = 1 ; 
P = 1;                  % Initial estimation covariance
% Initial state
roll_init = 0;         
pitch_init = 0;


% Compute the LKF for roll angle 
for k = 1:205

    % 1. KF Prediction
    x_pred = roll_init + u;
    P_pred = P + Q;

    % 2. KF Update
    G = P_pred * C' / (C * P_pred * C' + R);
    x = x_pred + G * (roll_new(k) - C * x_pred);
    P = (1 - G * C) * P_pred;

    % Store result
    roll_final(k) = x;
    P_vals(k) = P;
    fprintf('Sample %d -> Roll_last: %.2f deg\n', k, rad2deg(roll_final(k)));
    
    % Update roll_init for next iteration
    roll_init = x;
end

% Plot Gaussian curves at selected time steps
selected_k = [50, 100, 150, 200];
x_vals = linspace(min(roll_final)-3, max(roll_final)+3, 500);

figure;
hold on;
colors = lines(length(selected_k));
for i = 1:length(selected_k)
    mu = roll_final(selected_k(i));
    sigma = sqrt(P_vals(selected_k(i)));
    % Manual Gaussian PDF
    gauss = (1/(sqrt(2*pi)*sigma)) * exp(-((x_vals - mu).^2) / (2*sigma^2));
    plot(x_vals, gauss, 'Color', colors(i,:), 'DisplayName', sprintf('k = %d', selected_k(i)));
end
xlabel('State Value');
ylabel('Probability Density');
title('Gaussian Curves of Kalman Estimates for Roll Angle');
legend;
grid on;


% Compute the LKF for pitch angle 
for k = 1:205

    % 1. KF Prediction
    x_pred = pitch_init + u;
    P_pred = P + Q;

    % 2. KF Update
    G = P_pred * C' / (C * P_pred * C' + R);
    x = x_pred + G * (pitch_new(k) - C * x_pred);
    P = (1 - G * C) * P_pred;

    % Store result
    pitch_final(k) = x;
    P_vals(k) = P;
    fprintf('Sample %d -> Pitch_final: %.2f deg\n', k, rad2deg(pitch_final(k)));
    
    % Update pitch_init for next iteration
    pitch_init = x;
end

% Plot Gaussian curves at selected time steps
selected_k = [50, 100, 150, 200];
x_vals = linspace(min(pitch_final)-3, max(pitch_final)+3, 500);

figure;
hold on;
colors = lines(length(selected_k));
for i = 1:length(selected_k)
    mu = pitch_final(selected_k(i));
    sigma = sqrt(P_vals(selected_k(i)));
    gauss = (1/(sqrt(2*pi)*sigma)) * exp(-((x_vals - mu).^2) / (2*sigma^2));
    plot(x_vals, gauss, 'Color', colors(i,:), 'DisplayName', sprintf('k = %d', selected_k(i)));
end
xlabel('State Value');
ylabel('Probability Density');
title('Gaussian Curves of Kalman Estimates For Pitch Angle');
legend;
grid on;




