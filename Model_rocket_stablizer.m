% Model Rocket 1-Axis (Roll) Reaction Wheel Controller Simulation
clear; clc; close all;

%% 1. Define Physical Parameters (Inertias & Damping)
J_v = 0.0125;     % Rocket Roll Inertia (kg*m^2)
J_w = 0.000529;   % Reaction Wheel Inertia (kg*m^2)
b   = 0.002;      % Aerodynamic & Pivot Damping (N*m*s/rad)

%% 2. Propose Initial PID Controller Gains
Kp = 1772.21;        % Proportional Gain (Stiffness)
Ki = 2953.68;         % Integral Gain (Eliminates steady-state error)
Kd = 350.6;         % Derivative Gain (Damping/Braking)

%% 3. Construct the Plant Transfer Function P(s)
% P(s) = J_w / (J_v*s^2 + b*s)
num_P = [J_w];               
den_P = [J_v, b, 0];         
P = tf(num_P, den_P);        

%% 4. Construct the PID Controller Transfer Function C(s)
% C(s) = (Kd*s^2 + Kp*s + Ki) / s
num_C = [Kd, Kp, Ki];        
den_C = [1, 0];              
C = tf(num_C, den_C);        

%% 5. Construct the Closed-Loop System H(s)
H = feedback(C * P, 1);

%% 6. Graphing Results
figure('Name', 'Model Rocket Roll Stabilization Analysis', 'Position', [100, 100, 1100, 450]);

% --- Plot 1: Open-Loop Root Locus Map ---
subplot(1, 2, 1);
rlocus(C * P); 
title('Open-Loop Root Locus Map for Rocket C(s)P(s)');
xlabel('Real Axis (Seconds^{-1})');
ylabel('Imaginary Axis (Seconds^{-1})');
grid on;

% --- Plot 2: Simulated Step Response ---
subplot(1, 2, 2);
t = 0:0.005:5; % 5-second simulation time, calculated every 5 milliseconds
step(H, t);
title('Rocket Response to 1-Radian Roll Command');
xlabel('Time (seconds)');
ylabel('Roll Angle \phi (radians)');
grid on;