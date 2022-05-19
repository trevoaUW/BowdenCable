%Plots outputs from the controller code
%load('CapstoneDataNew.mat')
buffLen = 32500;
t = 0:BTILength:(buffLen-1)*BTILength;


%% Plotting with acceleration controller
%% Plot actual versus reference position
subplot(3,2,1)
% Experimental Data
plot(t, Curpos_on,'c:',LineWidth=2), hold on;
plot(t, Refpos_on, 'r:',LineWidth=2);

% Model Data
sim_source_on = [t', Refpos_on];
accel_model = sim("PID_discrete.slx");
model_on = accel_model.yout.getElement("theta");
model_pos_on = model_on.Values.Data;
model_t_on = model_on.Values.Time;
plot(model_t_on, model_pos_on, 'b:', LineWidth=2);

% Design conditions
pss_on = Refpos_on(buffLen,1);
yline(pss_on, 'k:');
OS = 1.1*pss_on; %overshoot
US = 0.9*pss_on; %undershoot
RT = 2; %settling time
yline(OS, 'm:');
yline(US, 'm:');
xline(RT, 'b:');

legend('p_a_c_t', 'p_r_e_f', 'p_t_h', Location='NorthWest');
title('Position Control Results');
%ylim([0,1.25*pss_on]);
%xlim([0,t(1,buffLen)]);
xlabel('Time (s)');
ylabel('Pos (rad)');
hold off;

%% Plot torque versus time
subplot(3,2,3);
plot(t, Torque_on, 'c:',LineWidth=2), hold on;
torque_bound = 1.1*max(Torque_on);
ylim([-torque_bound torque_bound]);
legend('actual torque',Location='SouthEast');
title('Controller Torque w/ Acc');
hold off;

%% Plot the output from the double derivative filter
subplot(3,2,5);
plot(t, Curaccel_on,'c:',LineWidth=2), hold on;
plot(t, Refaccel_on, 'r:',LineWidth=2);
legend('a_a_c_t', 'a_r_e_f', Location='SouthEast');
title('Output of Double Derivative Filter');
xlabel('Time (s)');
ylabel('Accel (rad/s^2)');
hold off;

%% Plots for no acceleration control scenario
%% Plot actual versus reference position
subplot(3,2,2)
plot(t, Curpos_off,'c:',LineWidth=2), hold on;
plot(t, Refpos_off, 'r:',LineWidth=2);

% Model Data
sim_source_off = [t', Refpos_off];
off_model = sim("PID_frict.slx");
model_off = off_model.yout.getElement("theta");
model_pos_off = model_off.Values.Data;
model_t_off = model_off.Values.Time;
plot(model_t_off, model_pos_off, 'b:', LineWidth=2);

% Design conditions
pss_off = Refpos_off(buffLen,1);
yline(pss_off, 'k:');
OS = 1.1*pss_off; %overshoot
US = 0.9*pss_off; %undershoot
RT = 2; %settling time
yline(OS, 'm:');
yline(US, 'm:');
xline(RT, 'b:');

legend('p_a_c_t', 'p_r_e_f', 'p_t_h', Location='NorthWest');
title('Position Control Results - no Acc');
%ylim([0,1.25*pss_off]);
%xlim([0,t(1,buffLen)]);
xlabel('Time (s)');
ylabel('Pos (rad)');
hold off;

%% Plot torque versus time
subplot(3,2,4);
plot(t, Torque_off, 'c:',LineWidth=2), hold on;
ylim([-torque_bound torque_bound]);
legend('actual torque',Location='SouthEast');
title('Controller Torque, no Acc');
hold off;

%% Run info
subplot(3,2,6);
nweights = 5; % MODIFY THIS BASED ON WEIGHTS USED!
material = 'Leather'; % MODIFY THIS BASED ON MATERIAL USED!
line1 = strcat('Ka = ', num2str(Ka));
line2 = strcat('Num weights: ', num2str(nweights));
line3 = strcat('Material: ',material);
info = {line1,line2,line3};
text(0.20,0.4,info); axis off
