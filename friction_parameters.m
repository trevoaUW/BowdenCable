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

% Loop to test static friction parameters
for i=0.1:0.1:1.1
    param.g4 = i;
    sim('plant_only_friction.slx');
    thetaSignal = ans.yout.getElement('theta');
    t = thetaSignal.Values.Time;
    theta = thetaSignal.Values.Data;
    hold on
    plot(t, theta)
    
end

xlabel('Time (s)')
ylabel('Theta (rad)')
title('Angular Position Response')
axis([0 10 0 10])


