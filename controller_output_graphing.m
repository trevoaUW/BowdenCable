%Plots outputs from the controller code
%load('CapstoneDataNew.mat')
buffLen = 32500;
t = 0:BTILength:(buffLen-1)*BTILength;

%% Plot actual versus reference position w/o Acc
subplot(4,2,1)
plot(t, Refpos_off, 'b-',LineWidth=2); hold on;
plot(t, Curpos_off,'r:',LineWidth=2); 

% Model Data
sim_source_off = [t', Refpos_off];
off_model = sim("PID_frict.slx");
model_off = off_model.yout.getElement("theta");
model_pos_off = model_off.Values.Data;
model_t_off = model_off.Values.Time;
plot(model_t_off, model_pos_off, 'g--', LineWidth=2);

legend('p_r_e_f', 'p_a_c_t', 'p_t_h', Location='NorthWest');
title('Position Control Results w/o Acc');
xlabel('Time (s)');
ylabel('Pos (rad)');
hold off;

%% Plot error versus time w/o Acc
subplot(4, 2, 3);
error_off = Refpos_off - Curpos_off;
plot(t, error_off, LineWidth=1);
error_bound = 1.1*max(abs(error_off));
yline(0, '--');
ylim([-error_bound error_bound]);
ylabel('Error (rad)');
xlabel('Time (s)');
title("Position Error w/o Acc");

%% Plot torque versus time w/o Acc
subplot(4,2,5);
plot(t, Torque_off, LineWidth=1), hold on;
torque_bound = 0.5; % Shared max torque
ylim([-torque_bound torque_bound]);
title('Controller Torque w/o Acc');
xlabel('Time (s)');
ylabel('Torque (Nm)');
hold off;

%% Plot actual versus reference position w/ Acc
subplot(4,2,2)
% Experimental Data
plot(t, Refpos_on, 'b-',LineWidth=2); hold on;
plot(t, Curpos_on,'r:',LineWidth=2); 

% Model Data
sim_source_on = [t', Refpos_on];
accel_model = sim("PID_discrete.slx");
model_on = accel_model.yout.getElement("theta");
model_pos_on = model_on.Values.Data;
model_t_on = model_on.Values.Time;
plot(model_t_on, model_pos_on, 'g--', LineWidth=2);

legend('p_r_e_f', 'p_a_c_t',  'p_t_h', Location='NorthWest');
title('Position Control Results w/ Acc');
xlabel('Time (s)');
ylabel('Pos (rad)');
hold off;

%% Plot error versus time w/ Acc
subplot(4, 2, 4);
error_on = Refpos_on - Curpos_on;
plot(t, error_on, LineWidth=1);
yline(0, '--');
ylim([-error_bound error_bound]);
title("Position Error w/ Acc");
ylabel('Error (rad)');
xlabel('Time (s)');

%% Plot torque versus time w/ Acc
subplot(4,2,6);
plot(t, Torque_on, LineWidth=1), hold on;
ylim([-torque_bound torque_bound]);
title('Controller Torque w/ Acc');
xlabel('Time (s)');
ylabel('Torque (Nm)')
hold off;

%% Plot the output from the double derivative filter w/ Acc
subplot(4,2,8);
plot(t, Curaccel_on,'b'), hold on;
plot(t, Refaccel_on, 'r');
legend('a_a_c_t', 'a_r_e_f', Location='SouthEast');
title('Output of Double Derivative Filter');
xlabel('Time (s)');
ylabel('Accel (rad/s^2)');
hold off;

%% Run info 
subplot(4,2,7);
nweights = 5; % MODIFY THIS BASED ON WEIGHTS USED!
material = 'Leather'; % MODIFY THIS BASED ON MATERIAL USED!
line1 = strcat('Ka = ', num2str(Ka));
line2 = strcat('Num weights: ', num2str(nweights));
line3 = strcat('Material: ',material);
info = {line1,line2,line3};
text(0.20,0.4,info, 'BackgroundColor', 'white', 'FontSize', 18); axis off
