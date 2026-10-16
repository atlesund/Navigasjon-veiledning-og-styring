function [phi_c,chi_int] = courseHold(chi,chi_c,Vg,chi_int,h)

% Add your controller gains and course hold PI controller here
w_n_phi = 0.5;
g = 9.81;
C_chi = 1;

wn_chi = (1/7.5) * w_n_phi;

kp_chi = 2*C_chi*wn_chi*(Vg/g);
ki_chi = wn_chi^2 * Vg/g;


phi_c = kp_chi*ssa(chi_c - chi) + ki_chi*chi_int;


% Euler's method: chi_int[k+1]
chi_int = chi_int + h * ssa(chi_c - chi);

end
