function delta_c = PID_heading(e_psi,e_r,e_int)
% Inputs:
%   e_psi: Error in yaw
%   e_r  : Error in roll
%   e_int: Accumulated error

T = 169.54;
%K = 0.007492;
m = 17.0677e6;          % mass (kg)
k = 0; % stiffness based on our approximated first order model

d = 1/T; %% eq 15.107 page 526 Fossen says m/T. (?)

w_b = 0.06; % bandwidth
Zeta = 1; % Relative damping ratio
w_n = w_b/sqrt(1-2*Zeta^2 + sqrt(4*Zeta^4 -4*Zeta^2 + 2));

Kp = m*w_n^2-k;
Kd = 2*Zeta*w_n*m-d;
Ki = w_n*Kp/10;

delta_c = Kp*e_psi + Kd*e_r + Ki*e_int;

end