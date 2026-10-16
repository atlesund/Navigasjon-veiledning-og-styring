function [delta_a_c,phi_int] = rollAttitudeHold(a_phi1,a_phi2,p,phi,phi_c,phi_int,h)

delta_a_max = 21;
e_phi_max = 15;
C_phi = 1;


% Add your controller gains and PID roll attitude controller here

kp_phi = sign(a_phi2) * delta_a_max/e_phi_max;

w_n_phi = sqrt(abs(a_phi2)*delta_a_max/e_phi_max);

kd_phi = (2*C_phi*w_n_phi - a_phi1)/(a_phi2);

ki_phi = kp_phi * w_n_phi/8;

e_phi = phi_c-phi;
delta_a_c = kp_phi*e_phi ...
            - kd_phi*p ...
            + ki_phi*phi_int;

% Euler's method: phi_int[k+1]
phi_int = phi_int + h * (phi_c - phi);

end
