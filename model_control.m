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
param.Ka = 470;
param.tau = 1/omega_b;
accel_cont = tf([param.Ka], [param.tau, 1]);

%% System Transfer Function + Find Root Locus
ol_accel = c*accel_cont*plant;
cl_accel = feedback(ol_accel, 1);

dot = tf([1 0], [1]);
double_dot = tf([1 0 0], [1]);

C = feedback(plant, dot);
D = feedback(accel_cont*param.J*C, double_dot);

rl_ol_tf = c*(1/param.J)*D;
rlocus(rl_ol_tf)

%% Simulation Loop
models = ["PID_no_frict.slx", "PID_frict.slx", "PID_accel_noise.slx", "PID_accel_reset.slx"];

for i = 1:3 
    % Vary Model
    model = sim(models(i));

    % Collect sim outputs
    thetaSignal = model.yout.getElement('theta');
    t_theta = thetaSignal.Values.Time;
    theta = thetaSignal.Values.Data;
    omegaSignal = model.yout.getElement('omega');
    t_omega = omegaSignal.Values.Time;
    omega = omegaSignal.Values.Data;
    errorSignal = model.yout.getElement('error');
    t_error = errorSignal.Values.Time;
    error = errorSignal.Values.Data;
    PIDTorqueSignal = model.yout.getElement('PID Torque');
    t_PID = PIDTorqueSignal.Values.Time;
    PID_torque = PIDTorqueSignal.Values.Data;
    
    % Plot Theta Response
    subplot(5,1,1)
    hold on
    plot(t_theta, theta, 'DisplayName', models(i))
    xlabel('Time (s)')
    ylabel('Theta (rad)')
    title('Angular Position')
    legend('Interpreter', 'none')

    % Plot error
    subplot(5,1,2)
    hold on
    plot(t_error, error, 'DisplayName', models(i));
    xlabel('Time (s)')
    ylabel('Error (rad)')
    title('Error Signal')
    legend('Interpreter', 'none')

    % Plot Omega Response
    subplot(5,1,3)
    hold on
    plot(t_omega, omega, 'DisplayName', models(i))
    xlabel('Time (s)')
    ylabel('Omega (rad/s)')
    title('Angular Velocity')
    legend('Interpreter', 'none')

    % Plot PID Torque Response
    subplot(5,1,4)
    hold on
    plot(t_PID, PID_torque, 'DisplayName', models(i))
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    title('PID Torque')
    legend('Interpreter', 'none')
    
    % Plot Accel Torque Response
%     subplot(5,1,5)
%     hold on
%     plot(t_accel, Accel_torque, 'DisplayName', ['i=' num2str(i)])
%     xlabel('Time (s)')
%     ylabel('Torque (Nm)')
%     title('Accel. Controller Torque')
%     legend()

    % Plot max theta
%     subplot(4,1,3)
%     hold on
%     max_theta = max(theta);
%     plot(i, max_theta, 'o');
%     xlabel('Coulomb Friction Coeff')
%     ylabel('Theta (rad)')
%     title('Max Theta')
end

% Plot Accel Torque Response

AccelTorqueSignal = model.yout.getElement('Accel Torque');
t_accel = AccelTorqueSignal.Values.Time;
Accel_torque = AccelTorqueSignal.Values.Data;

subplot(5,1,5)
hold on
plot(t_accel, Accel_torque, 'DisplayName', ['i=' num2str(i)])
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Accel. Controller Torque')
legend()

%% Plot Friction curve
u = -10:0.01:10;
frict_force = friction_m(u, param);
figure(2)
plot(u, frict_force)
xlabel("Velocity")
ylabel("Friction Force")
title("Friction Curve")

%% Functions
function f = friction_m(u, param)

%FRICTION_M Nonlinear friction model with Stribeck, Coulomb and viscous

% dissipation effects.

% Output equation.

f = param.g1*(tanh(param.g2*u)-tanh(param.g3*u)) ... % Stribeck effect.
+param.g4*tanh(param.g5*u) ... % Coulomb effect.
+ param.g6*u; % Viscous dissipation term.

end



