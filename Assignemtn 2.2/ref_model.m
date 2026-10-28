function xd_dot = ref_model(xd,psi_ref)
% Generate the desired heading (psi) and yaw rate (r_d)
% states = [psi_d, r_d, rdot_d]

wn = 0.03;

psi_d = xd(1);
r_d = xd(2);
r_dot_d = xd(3);

xd_dot = [
    r_d;
    r_dot_d;
    wn^3*(psi_ref-psi_d) - 3*wn^2*r_d - 3*wn*r_dot_d % third derivative
    ];


end