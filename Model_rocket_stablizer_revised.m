clear; clc; close all;

J_v = 0.0125;
J_w = 0.002535;
b = 0.002;

Kp = 369.82;
Ki = 616.37;
Kd = 73.18;

num_P = [J_w];
den_P = [J_v, b, 0];
P = tf(num_P, den_P);

num_C = [Kd, Kp, Ki];
den_C = [1, 0];
C = tf(num_C, den_C);

H = feedback(C * P, 1);


subplot(1, 2, 1);
rlocus(C * P); 
title('Open-Loop Root Locus Map for Rocket C(s)P(s)');
xlabel('Real Axis (Seconds^{-1})');
ylabel('Imaginary Axis (Seconds^{-1})');
grid on;

subplot(1, 2, 2);
t = 0:0.005:5;
step(H, t);
title('Rocket Response to 1-Radian Roll Command');
xlabel('Time (seconds)');
ylabel('Roll Angle \phi (radians)');
grid on;