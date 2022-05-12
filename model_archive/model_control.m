%% Simulation Loop
models = ["PID_no_frict.slx", "PID_frict.slx", "PID_accel.slx"];

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
plot(t_accel, Accel_torque)
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



