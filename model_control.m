clear
clc
close all

%% Parameters

% Friction Parameters
param.g1 = 5;    % max stiction value
param.g2 = 90;
param.g3 = 11;
param.g4 = 3;    % max Coulomb value
param.g5 = 100;
param.g6 = 0;    % damping coeff.

% Plant Parameters
param.J = 1e-2;

%% PID Controller Design
plant = tf([1], [param.J 0 0]);
omega_c = 6; % rad/s;
opt = pidtuneOptions('DesignFocus', 'reference-tracking');
c = pidtune(plant, 'pidf', omega_c, opt);

param.Kp = c.Kp;
param.Ki = c.Ki;
param.Kd = c.Kd;
param.N = 1/c.Tf;

models = ["PID_no_frict.slx", "PID_frict.slx", "PID_accel.slx", "PID_accel_reset.slx"];

%% Simulation Loop

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
%     AccelTorqueSignal = model.yout.getElement('Accel Torque');
%     t_accel = AccelTorqueSignal.Values.Time;
%     Accel_torque = AccelTorqueSignal.Values.Data;

    % Plot Theta Response
    subplot(4,1,1)
    hold on
    plot(t_theta, theta, 'DisplayName', models(i))
    xlabel('Time (s)')
    ylabel('Theta (rad)')
    title('Angular Position')
    legend('Interpreter', 'none')

    % Plot error
    subplot(4,1,2)
    hold on
    plot(t_error, error, 'DisplayName', models(i));
    xlabel('Time (s)')
    ylabel('Error (rad)')
    title('Error Signal')
    legend('Interpreter', 'none')

    % Plot Omega Response
    subplot(4,1,3)
    hold on
    plot(t_omega, omega, 'DisplayName', models(i))
    xlabel('Time (s)')
    ylabel('Omega (rad/s)')
    title('Angular Velocity')
    legend('Interpreter', 'none')

    % Plot PID Torque Response
    subplot(4,1,4)
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

%% Plot Friction curve
u = -10:0.01:10;
frict_force = friction_m(u, param);
figure(2)
plot(u, frict_force)
axis([-10 10 -5 5])
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



