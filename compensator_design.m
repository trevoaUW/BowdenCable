clc
clear 
clf

%% System Parameters
J = 2;
sys = tf(1, [J 0 0])
cont = zpk(-3, -120, 1)
cont_sys = cont*sys
rlocus(cont_sys)