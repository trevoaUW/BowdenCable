clear
clc
close all

%% Parameters

% Friction Parameters
param.g1 = 0.1;    % max stiction value
param.g2 = 90;
param.g3 = 11;
param.g4 = 0.05;    % max Coulomb value
param.g5 = 100;
param.g6 = 0;    % damping coeff.

% Plant Parameters
param.J = 0.00034;

%% PID Controller Design
plant = tf([1], [param.J 0 0]);
omega_c = 15; % rad/s;
opt = pidtuneOptions('DesignFocus', 'reference-tracking');
c = pidtune(plant, 'pidf', omega_c, opt);

param.Kp = c.Kp;
param.Ki = c.Ki;
param.Kd = c.Kd;
param.N = 1/c.Tf;

%% Acceleration Controller Design
ol_no_accel = c*plant;
cl_no_accel = feedback(ol_no_accel, 1);
omega_b = 5*bandwidth(cl_no_accel);
param.Ka = 50;
param.tau = 1/omega_b;
accel_cont = tf([param.Ka], [param.tau, 1]);

%% Accel Root Locus
ol_accel = c*accel_cont*plant;
cl_accel = feedback(ol_accel, 1);

dot = tf([1 0], [1]);
double_dot = tf([1 0 0], [1]);

C = feedback(plant, dot);
D = feedback(accel_cont*param.J*C, double_dot);

rl_ol_tf = c*(1/param.J)*D;
whole_sys_tf = feedback(rl_ol_tf, 1);
