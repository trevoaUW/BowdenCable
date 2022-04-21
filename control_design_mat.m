%ME 495 Bowden Cable Capstone
clear all; close all;clc;
% Friction Parameters
param.g1 = 0.1;    % max stiction value
param.g2 = 90;
param.g3 = 11;
param.g4 = 0.05;    % max Coulomb value
param.g5 = 100;
param.g6 = 0;    % damping coeff.

% Plant Parameters
param.J = 0.00034;
T = 0.005; %s

%% PID Controller Design
plant = tf([1], [param.J 0 0]);
omega_c = 15; % rad/s;
opt = pidtuneOptions('DesignFocus', 'reference-tracking');
pid = pidtune(plant, 'pidf', omega_c, opt)

param.Kp = pid.Kp;
param.Ki = pid.Ki;
param.Kd = pid.Kd;
param.N = 1/pid.Tf;

%convert to discrete form
cdp = c2d(pid,T,'tustin'); %parallel discrete
cd = tf(cdp); %transfer function form
[b,a] = tfdata(cd, 'v'); %outputs to array
sos = tf2sos(b,a);

%Send to location where Eclipse will be able to access it
%%%NEED TO UPDATE LOCATION FOR USE IN TESTing
fid = fopen("C:/Users/Natalie/myLab8/myPIDF.h", 'w');
comment = 'Friction Capstone Controllers';
sos2header(fid, sos, 'PIDF',T, comment);

%% DOUBLE DERIVATIVE FILTER
%TBD

% %convert with Tustin
% cdp = c2d(dderiv,T,'tustin'); %parallel discrete
% cd = tf(cdp); %transfer function form
% [b,a] = tfdata(cd, 'v'); %outputs to array
% sos = tf2sos(b,a);
% 
% %Send to location where Eclipse will be able to access it
% %%%NEED TO UPDATE LOCATION FOR USE IN TESTing
% fid = fopen("C:/Users/Natalie/myLab8/myDDeriv.h", 'w');
% comment = 'Double Derivative Filter';
% sos2header(fid, sos, 'DDERIV',T, comment);

%% ACCELERATION CONTROLLER
ol_no_accel = pid*plant;
cl_no_accel = feedback(ol_no_accel, 1);
omega_b = 5*bandwidth(cl_no_accel);
param.Ka = 12;
param.tau = 1/omega_b;
accel_cont = tf([param.Ka], [param.tau, 1]);

%convert with Tustin
cdp = c2d(accel_cont,T,'tustin'); %parallel discrete
cd = tf(cdp); %transfer function form
[b,a] = tfdata(cd, 'v'); %outputs to array
sos = tf2sos(b,a);

%Send to location where Eclipse will be able to access it
%%%NEED TO UPDATE LOCATION FOR USE IN TESTing
fid = fopen("C:/Users/Natalie/myLab8/myAcc.h", 'w');
comment = 'Acceleration Controller';
sos2header(fid, sos, 'ACC',T, comment);

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
  function sos2header(fid, sos, name, T, comment)
  
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
fprintf(fid,'    char        headerTime[] = "%s";\n',datestr(now,0));
[ns,m]=size(sos);
fprintf(fid,'    int         %s_ns = %d;              // number of sections\n',name,ns);
fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
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