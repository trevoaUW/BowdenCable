clear
clc
close all

% Friction Parameters
param.g1 = 0;
param.g2 = 0;
param.g3 = 0;
param.g4 = 1000;
param.g5 = 100;
param.g6 = 0.1;
param.J = 1;

% Run simulation
sim('plant_only_friction.slx');

% Angular Position Reponse
thetaSignal = ans.yout.getElement('theta');
t = thetaSignal.Values.Time;
theta = thetaSignal.Values.Data;
hold on
figure(1)
plot(t, theta)
xlabel('Time (s)')
ylabel('Theta (rad)')
title('Angular Position Response')
axis([0 10 0 10])

% Plot Friction curve
u = -10:0.01:10;
frict_force = friction_m(u, param);
figure(2)
plot(u, frict_force)
axis([-10 10 -5 5])
xlabel("Velocity")
ylabel("Friction Force")
title("Friction Curve")

function f = friction_m(u, param)

%FRICTION_M Nonlinear friction model with Stribeck, Coulomb and viscous

% dissipation effects.

% Output equation.

f = param.g1*(tanh(param.g2*u)-tanh(param.g3*u)) ... % Stribeck effect.
+param.g4*tanh(param.g5*u) ... % Coulomb effect.
+ param.g6*u; % Viscous dissipation term.

end



