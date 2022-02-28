clear
clc
close all

% Friction Parameters
param.g1 = 0;
param.g2 = 0;
param.g3 = 0;
param.g4 = 0;
param.g5 = 100;
param.g6 = 0;
param.J = 1;

%% Simulation Loop
for i = 0:5:20
    % Vary Coulomb Friction Coefficient
    param.g4 = i;
    sim('PID_Control_acceleration_loop.slx');

    % Collect sim outputs
    thetaSignal = ans.yout.getElement('theta');
    t_theta = thetaSignal.Values.Time;
    theta = thetaSignal.Values.Data;
    omegaSignal = ans.yout.getElement('omega');
    t_omega = omegaSignal.Values.Time;
    omega = omegaSignal.Values.Data;
    errorSignal = ans.yout.getElement('error');
    t_error = errorSignal.Values.Time;
    error = errorSignal.Values.Data;

    % Plot Theta Response
    subplot(2,2,1)
    hold on
    plot(t_theta, theta)
    xlabel('Time (s)')
    ylabel('Theta (rad)')
    title('Angular Position Response')

    % Plot Omega Response
    subplot(2,2,2)
    hold on
    plot(t_omega, omega)
    xlabel('Time (s)')
    ylabel('Omega (rad/s)')
    title('Angular Velocity Response')

    % Plot max theta
    subplot(2,2,3)
    hold on
    max_theta = max(theta);
    plot(i, max_theta, 'o');
    xlabel('Coulomb Friction Coeff')
    ylabel('Theta (rad)')
    title('Max Theta')

    % Plot error
    subplot(2,2,4)
    hold on
    plot(t_error, error);
    xlabel('Time (s)')
    ylabel('Error (rad)')
    title('Error Signal')
end

%% Plot Friction curve
% u = -10:0.01:10;
% frict_force = friction_m(u, param);
% figure(2)
% plot(u, frict_force)
% axis([-10 10 -5 5])
% xlabel("Velocity")
% ylabel("Friction Force")
% title("Friction Curve")

%% Functions
function f = friction_m(u, param)

%FRICTION_M Nonlinear friction model with Stribeck, Coulomb and viscous

% dissipation effects.

% Output equation.

f = param.g1*(tanh(param.g2*u)-tanh(param.g3*u)) ... % Stribeck effect.
+param.g4*tanh(param.g5*u) ... % Coulomb effect.
+ param.g6*u; % Viscous dissipation term.

end



