%ME 495 Bowden Cable Capstone
% clear all; close all;clc;
% Friction Parameters
param.g1 = 0;    % Stiction is suppressed
param.g2 = 0;
param.g3 = 0;
param.g4 = 0.1752;    % max Coulomb value
param.g5 = 100;
param.g6 = 0.0023;    % damping coeff.

% Plant Parameters
param.J = 0.00034;
T = 0.001; %s
Hz = 1/T;

%% PID Controller Design
plant = tf([1], [param.J 0 0]);
omega_c = 15; % rad/s;
opt = pidtuneOptions('DesignFocus', 'reference-tracking');
pid = pidtune(plant, 'pidf', omega_c, opt);

param.Kp = pid.Kp;
param.Ki = pid.Ki;
param.Kd = pid.Kd;
param.N = 1/pid.Tf;

% convert to discrete form
cdp_pid = c2d(pid,T,'tustin'); %parallel discrete
cd_pid = tf(cdp_pid); %transfer function form
[b_pid, a_pid] = tfdata(cd_pid, 'v'); %outputs to array
sos = tf2sos(b_pid, a_pid);

% Send to location where Eclipse will be able to access it
% NEED TO UPDATE LOCATION FOR USE IN TESTing
fid = fopen("C:\Users\Trevor\Documents\UW Files\ME 477\workspace\myLab8\myPIDF.h", 'w');
comment = 'Friction Capstone Controllers';
sos2header(fid, sos, 'PIDF',T, 0, comment);

%% DOUBLE DERIVATIVE FILTER
w_b_factor = 0.0075;
tau_d = 1/(w_b_factor*Hz*2*pi);
H1 = tf([1 0],[tau_d 1]);
H2 = tf([1 0],[tau_d, 1]);
dderiv = series(H1, H2);

%convert with Tustin
cdp_dd = c2d(dderiv,T,'tustin'); %parallel discrete
cd_dd = tf(cdp_dd); %transfer function form
[b_dd, a_dd] = tfdata(cd_dd, 'v'); %outputs to array
sos2 = tf2sos(b_dd, a_dd);

% Send to location where Eclipse will be able to access it
% NEED TO UPDATE LOCATION FOR USE IN TESTING
fid = fopen("C:\Users\Trevor\Documents\UW Files\ME 477\workspace\myLab8\myDDeriv.h", 'w');
comment2 = 'Double Derivative Filter';
sos2header(fid, sos2, 'DDERIV',T, tau_d, comment2);

%% ACCELERATION CONTROLLER
ol_no_accel = pid*plant;
cl_no_accel = feedback(ol_no_accel, 1);
omega_b = 5*bandwidth(cl_no_accel);
param.Ka = 5;  % Changes model parameters but not controller Ka
param.tau = 1/omega_b;
accel_cont = tf([param.Ka], [param.tau, 1]); % Adjust Ka on keypad

% convert with Tustin
cdp_ac = c2d(accel_cont,T,'tustin'); %parallel discrete
cd_ac = tf(cdp_ac); %transfer function form
[b_ac, a_ac] = tfdata(cd_ac, 'v'); %outputs to array
sos3 = tf2sos(b_ac, a_ac);

%Send to location where Eclipse will be able to access it
%%%NEED TO UPDATE LOCATION FOR USE IN TESTing
fid = fopen("C:\Users\Trevor\Documents\UW Files\ME 477\workspace\myLab8\myAcc.h", 'w');
comment3 = 'Acceleration Controller';
sos2header(fid, sos3, 'ACC',T, param.tau, comment3);

% %Test controller (discrete)
% OLTF_d = series(Hd, cdp);
% CLTF_d = feedback(OLTF_d, 1);
% T_CL_d = feedback(cd, Hd);
% 
% %Test controller (continuous)
% OLTF_c = series(H, c);
% CLTF_c = feedback(OLTF_c, 1);
% T_CL_c = feedback(c, H);
% 
% %plot responses
% subplot(2,1,1)
% step(CLTF_c, CLTF_d, 50*T);
% title("Position: X/Xref")
% subplot(2,1,2)
% step(T_CL_c, T_CL_d, 50*T);
% title("Torque: T/Xref")
% figure
% bode(OLTF_c)
% margin(OLTF_c)
  function sos2header(fid, sos, name, T, tau, comment)
  
  % Print the filter definition to a (.h) header file.
  %
  %   sos2header(fid, sos, name, T, comment)
  %
  %    fid      - File indentity
  %    sos      - Scaled second order sections, from "tf2sos"
  %    name     - Name to be given to the array of biquad structures, and
  %                  associated with the number of sections.
  %    T        - Sample period in seconds
  %    comment  - comment added at top of header
  
%---structure form of cascade

fprintf(fid,'//---%s\n', comment);
fprintf(fid,'//---%s\n', datestr(now,0));
%fprintf(fid,'    char        headerTime[] = "%s";\n',datestr(now,0));
%%commented out for now, causing redundancy issues
[ns,m]=size(sos);
fprintf(fid,'    int         %s_ns = %d;              // number of sections\n',name,ns);
if strcmp(name, 'DDERIV')
    fprintf(fid,'    double tau_DD = %f;   // tau\n', tau);
end
if strcmp(name, 'ACC')
    fprintf(fid,'    double tau_ACC = %f;   // tau\n', tau);
end
if strcmp(name,'PIDF')
    fprintf(fid,'    double  BTI = %f;     // sample period\n', T);
    fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
end
fprintf(fid,'    static\tstruct\tbiquad %s[]={   // define the array of floating point biquads\n',name);
for i=1:ns-1
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(i,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0},\n');
end
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(ns,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');
  end