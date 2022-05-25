%% Pull the values we need from data
% Static friction
%find index where vact>0
%check next 2-3 indices and confirm they're also >0
%grab that value
V_T = [V_max: T_max];
tol = 100; %RPM
for i=1:1:len(V_T)-1
    v_0 = V_T(1, i);
    v_1 = V_T(1, i+1);
    v_2 = V_T(1, i+2);
    if v_0>tol && v_1>tol && v_2>tol %making sure it's moving
        ind_stat = i;
        break
    end
end
stat_val = V_T(:, ind_stat)
%still need to turn this into coeffiecnt(s)

%calculation of force on system (to back-calculate us)
R = 0.0318; %m
n = 1; %number of masses used
m_weight = 107*n;
m_arm = -93.2; %empty arm
m_inserts = [20.4 25.6]; %leather rubber
m = (m_arm + m_inserts + m_masses)/1000;
F = m*g;

us_exp = stat_val(ind_stat)/(F*R)

%% Finding kinetic friction via slope
range_k = V_T(:,i:end); %presumably vals after this are "kinetic"
vprev = V_T(1,1);
for j=1:1:length(range_k)-1
    vcurr = V_T(1,j);
	error = abs(vcurr-vprev); 
    if error < 0.03*vcurr %within a given threshold
        ind_k = j; %we are at steady state!
        break
    end
end
%all_kvals = range_k(:,k:);
%find slope
%x = all_kvals(1,:);
%y = all_kvals(2,:);
%P = polyfit(x,y,1);
%u_k = P(1) %gives us the slope!

function f = friction_m(u, param)
    %FRICTION_M Nonlinear friction model with Stribeck, Coulomb and viscous
    % dissipation effects.
    % Output equation.
    f = param.g1*(tanh(param.g2*u)-tanh(param.g3*u)) ... % Stribeck effect.
    + param.g4*tanh(param.g5*u) ... % Coulomb effect.
    + param.g6*u; % Viscous dissipation term.
end