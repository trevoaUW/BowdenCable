%% Testing Suite Output Analysis file
%clockwise is positive
%clear all; close all; clc;

%%Clean data
% actualVels
% torques
% current_ref_velocity
% previous_ref_velocity
% Kp
% Ki
% BTI
t = 0:BTI:(length(actualVels)-1)*BTI;

%% PLOTS
%plot: V over time
subplot(3,1,1);
plot(t, actualVels,":",'LineWidth', 2), hold on;
ylim([1.1*min(actualVels) 1.1*max(actualVels)]);
xlim([0 1.1*max(t)]);
title('Velocity over Test Duration');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
resolution = 60./(BTI * 2000);
target_v = 10;
hold off;
 
%plot: T over time
subplot(3,1,2);
plot(t, torques,":",'LineWidth', 2), hold on;
ylim([-0.5 0.5]); %based on assumed max torque; change at higher loads
xlim([1.1*min(t) 1.1*max(t)]);
yline(max(torques), 'k:');
yline(min(torques), 'k:')
title('Torque over Test Duration');
xlabel('Time (s)');
ylabel('Torque (Nm)');
hold off;

%plot: torque vs v curve
subplot(3,1,3);
plot(actualVels, torques, 'r.'), hold on;
ylim([-0.2 0.2]); %based on assumed max torque; change at higher loads
xlim([1.1*min(actualVels) 1.1*max(actualVels)]);

%%Curve fitting the friction model
xdata = actualVels;
ydata = torques;
g = [0.0478 100 0];
f = @(g, xdata) g(1)*tanh(g(2)*xdata) + g(3)*xdata;
%g = [0 90 11 0.0478 100 0];
%f = @(g, xdata)g(1)*tanh(g(2)*xdata) - tanh(g(3)*xdata)...
    %+g(4)*tanh(g(5)*xdata) + g(6)*xdata;
x = lsqcurvefit(f, g, xdata, ydata)
predicted_curve = f(x, ref_vel_array);
plot(ref_vel_array, predicted_curve, 'b-','LineWidth', 2)

title('Friction Curve for System');
xlabel('Velocity (rad/s)');
ylabel('Torque (N/m)');
legend('Actual curve','Theory curve',Location='SouthEast');
hold off;

