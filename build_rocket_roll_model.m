%% build_rocket_roll_model.m
% Builds and saves "rocket_roll_model.slx" - roll stabilization loop
% using only primitive blocks (no PID / Transfer Fcn blocks).
%
%   C(s) = Kd*s + Kp + Ki/s         (parallel P + I + D, built from Gain/Integrator/Derivative)
%   P(s) = J_w / (J_v*s^2 + b*s)    (built as two integrators + damping feedback)
%
% Run this script once. It creates, wires, saves and opens the model.

clear; clc; close all;

mdl = 'rocket_roll_model';

%% Parameters (also stored inside the model so it runs standalone)
paramCode = [ ...
    'J_v = 0.0125; J_w = 0.002535; b = 0.002; ' ...
    'Kp = 369.82; Ki = 616.37; Kd = 73.18;'];
eval(paramCode);

%% Create model
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'PreLoadFcn', paramCode, 'InitFcn', paramCode);

% Fixed-step solver: the ideal derivative of a step command is an impulse,
% a small fixed step approximates it consistently (matches step(H) closely).
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '1e-4', 'StopTime', '5');

%% Helper
add = @(src, name, pos, varargin) add_block(src, [mdl '/' name], ...
    'Position', pos, varargin{:});

%% Blocks - reference and error
add('simulink/Sources/Step', 'Roll_Cmd', [30 148 60 178], ...
    'Time', '0', 'Before', '0', 'After', '1');
add('simulink/Math Operations/Sum', 'Error_Sum', [110 145 130 175], ...
    'Inputs', '|+-', 'IconShape', 'round');

%% Blocks - controller (Kp + Ki/s + Kd*s)
add('simulink/Math Operations/Gain', 'Kp_Gain', [230 60 280 90], 'Gain', 'Kp');
add('simulink/Continuous/Integrator', 'Ki_Integrator', [200 150 230 180]);
add('simulink/Math Operations/Gain', 'Ki_Gain', [255 150 305 180], 'Gain', 'Ki');
add('simulink/Continuous/Derivative', 'D_Derivative', [200 240 240 270]);
add('simulink/Math Operations/Gain', 'Kd_Gain', [265 240 315 270], 'Gain', 'Kd');
add('simulink/Math Operations/Sum', 'Ctrl_Sum', [370 140 390 200], ...
    'Inputs', '+++', 'IconShape', 'rectangular');

%% Blocks - plant: J_v*phi'' + b*phi' = J_w*u
add('simulink/Math Operations/Gain', 'Jw_Gain', [440 148 490 178], 'Gain', 'J_w');
add('simulink/Math Operations/Sum', 'Torque_Sum', [530 145 550 175], ...
    'Inputs', '+-', 'IconShape', 'round');
add('simulink/Math Operations/Gain', 'InvJv_Gain', [585 148 635 178], 'Gain', '1/J_v');
add('simulink/Continuous/Integrator', 'Omega_Integrator', [670 148 700 178]);   % roll rate
add('simulink/Continuous/Integrator', 'Phi_Integrator', [750 148 780 178]);     % roll angle
add('simulink/Math Operations/Gain', 'b_Gain', [585 230 635 260], 'Gain', 'b', ...
    'Orientation', 'left');

%% Blocks - outputs
add('simulink/Signal Routing/Mux', 'Scope_Mux', [850 120 855 200], 'Inputs', '2');
add('simulink/Sinks/Scope', 'Roll_Scope', [890 145 920 175]);
add('simulink/Sinks/To Workspace', 'phi_ToWS', [850 250 910 280], ...
    'VariableName', 'phi_out', 'SaveFormat', 'Array');

%% Wiring
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');

L('Roll_Cmd/1',        'Error_Sum/1');
% error -> three paths
L('Error_Sum/1',       'Kp_Gain/1');
L('Error_Sum/1',       'Ki_Integrator/1');
L('Error_Sum/1',       'D_Derivative/1');
L('Ki_Integrator/1',   'Ki_Gain/1');
L('D_Derivative/1',    'Kd_Gain/1');
% sum controller terms
L('Kp_Gain/1',         'Ctrl_Sum/1');
L('Ki_Gain/1',         'Ctrl_Sum/2');
L('Kd_Gain/1',         'Ctrl_Sum/3');
% plant
L('Ctrl_Sum/1',        'Jw_Gain/1');
L('Jw_Gain/1',         'Torque_Sum/1');
L('Torque_Sum/1',      'InvJv_Gain/1');
L('InvJv_Gain/1',      'Omega_Integrator/1');
L('Omega_Integrator/1','Phi_Integrator/1');
% damping feedback (omega -> b -> torque sum)
L('Omega_Integrator/1','b_Gain/1');
L('b_Gain/1',          'Torque_Sum/2');
% unity feedback of roll angle
L('Phi_Integrator/1',  'Error_Sum/2');
% outputs
L('Roll_Cmd/1',        'Scope_Mux/1');
L('Phi_Integrator/1',  'Scope_Mux/2');
L('Scope_Mux/1',       'Roll_Scope/1');
L('Phi_Integrator/1',  'phi_ToWS/1');

%% Annotation
add_block('built-in/Note', [mdl '/Note'], 'Position', [30 20], ...
    'Text', 'Roll stabilization: C = Kp + Ki/s + Kd*s,  P = J_w/(J_v s^2 + b s)');

%% Save
save_system(mdl, [mdl '.slx']);
fprintf('Saved %s.slx\n', mdl);

%% Optional check: compare against the transfer-function result
simOut = sim(mdl);
phi_sim = simOut.phi_out;
t_sim   = simOut.tout;

P = tf(J_w, [J_v b 0]);
C = tf([Kd Kp Ki], [1 0]);
H = feedback(C*P, 1);
[y, t] = step(H, 0:0.005:5);

figure('Name', 'Simulink vs tf step response');
plot(t, y, 'LineWidth', 2); hold on;
plot(t_sim, phi_sim, '--', 'LineWidth', 1.5); grid on;
legend('step(H)', 'Simulink model');
xlabel('Time (s)'); ylabel('Roll angle \phi (rad)');
title('Rocket response to 1-rad roll command');
