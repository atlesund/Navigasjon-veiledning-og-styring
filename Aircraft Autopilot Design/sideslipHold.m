function [delta_r_c, beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h)

% Add your controller gains and sideslip PI controller here
delta_r_max = 30;
e_B_max = 5;
C_B = 1;

kp_B = sign(a_beta2) *delta_r_max/e_B_max;
ki_B = (1/a_beta2)*((a_beta1 + a_beta2*kp_B)/(2*C_B))^2;



delta_r_c = -kp_B*beta - beta_int * ki_B;

% Euler's method: beta_int[k+1]
beta_int = beta_int + h * beta;

end