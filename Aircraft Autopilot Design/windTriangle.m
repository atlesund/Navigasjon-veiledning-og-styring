function [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)
% [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)

% Add your code here
Vg = sqrt(sum(v_g.^2)); 
Vw = sqrt(sum(v_w.^2));

v_a = v_g - v_w;
Va = sqrt(sum(v_a.^2));

alpha = rad2deg(atan2(v_a(3), v_a(1)));
beta = rad2deg(atan2(v_a(2), v_a(1)));

end