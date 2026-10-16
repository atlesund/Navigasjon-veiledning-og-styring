%% openLoop script for testing the F16 stability properties

D2R = pi / 180;     % degrees to radians
R2D = 180 / pi;     % radians to degrees
g = 9.81;           % acceleration of gravity [m/s^2]

v_g = [257.9 0 0]';       % ground velocity vector expressed in BODY [m/s]
v_w = [14.3 0.5 -0.3]';  % wind velocity vector expressed in BODY [m/s]
Vg = norm(v_g);

% Aircraft model: x = [beta p r phi psi]' and u = [ delta_a delta_r ]'
A = [ -0.1203 0.1479 -0.9860 0.0636 0
      -17.5184 -1.2523 0.5484 0 0
       3.5318 -0.0274 -0.1788 0 0
       0 1 0.1489 0 0 
       0 0 1 0 0];
 
B = [ 0.0001 0.0003 
     -0.2560 -0.0044
     -0.0133 -0.0292
      0 0
      0 0 ];

a_phi1 = -A(2,2);              % roll transfer function 
a_phi2 =  B(2,1);              % p_dot = -a_phi1 * p + a_phi2 * delta_a

disp(A) 

% Transfer functions
s = tf('s');
Gphi = a_phi2/(s+a_phi1) * (1/s)  % transfer function from aileron to phi
Gchi = Gphi * (g/(Vg*s))          % transfer function from aileron to chi
 
% Bode plot - open loop
clf; figure(1); figure(gcf)
opts = bodeoptions('cstprefs');
opts.XLim = [0.001 100];
bode(Gphi,'b',Gchi,'r',opts);
legend('\phi/\delta_a', '\chi/\delta_a','FontSize',14);
title('Bode diagram of the lateral aircraft dynamics (open loop)'), grid 
set(findall(gcf,'type','line'),'linewidth',1.5)

% Eigenvalues 
figure(2); figure(gcf)
plot(eig(A(1:4,1:4)),'bx','MarkerSize',12,'linewidth',2),grid
damp(A(1:4,1:4))

% Wind triangle computations
[alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)
