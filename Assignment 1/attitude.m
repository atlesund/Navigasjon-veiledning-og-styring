% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions using a MAIN LOOP with fixed step. 
% Euler's method, RK4, and exact discetization are investigated.
% 
% The MSS m-files must be on your Matlab path in order to run the script.
% See How to install Matlab for MSS: https://github.com/cybergalactic/MSS/
%
% System:                      
%                            q_dot = T(q)w
%                             
%                            Ib * w_dot - S(Ib * w) * w = tau
% Control law:
%                            tau = constant (3x1)
% 
% Definitions:             
%                            Ib : inertia matrix (3x3)
%                            S(w) : Skew-symmetric matrix (3x3)
%                            T(q) : transformation matrix (4x3)
%                            tau : Control input (3x1)
%                            w : Angular velocity vector (3x1)
%                            q : unit quaternion vector (4x1)
%% USER INPUTS
clc; clear;




T_final = 400;	             % Final simulation time (s)
h = 0.1;                     % Sampling time (s)

% Model parameters
m = 10;                      % Mass
R = 0.1 / sqrt(6);           % Radius of gyration
Ib = m * R^2 * eye(3);       % Inertia matrix in CO
I_inv = invQR(Ib);           % Using the highly accurate QR decomposition method to compute the matrix inverse

% Initial states
phi = -deg2rad(5);          % Initial Euler angles
theta = deg2rad(10);
psi = -deg2rad(20);

q = euler2q(phi,theta,psi);  % Transform initial Euler angles to q

w = [0 0 0]';                % Initial angular rates

% Regulator
%kp = 5.0e-04;
%kd = 9.0e-03;
kp = 5.0e-03;
kd = 9.0e-02;

% Time vector initialization
t = 0:h:T_final;                % Time vector from 0 to T_final          
nTimeSteps = length(t);         % Number of time steps

%% MAIN LOOP
simdata = zeros(nTimeSteps, 16); % Pre-allocate table for simdata

syms ts % Using symbolic expressions for desired theta and psi.
theta_d = @(ts) deg2rad(15*cos(0.1*ts));
psi_d   = @(ts) deg2rad(10*sin(0.05*ts));

dtheta_d = @(ts) deg2rad(-1.5*sin(0.1*ts));
dpsi_d   = @(ts) deg2rad(0.5*cos(0.05*ts));

for i = 1:nTimeSteps

    % Desired time-varying states
    phi_desired = 0;
    theta_desired = theta_d(t(i));
    psi_desired = psi_d(t(i));

    phi_dot_desired = 0;
    theta_dot_desired = dtheta_d(t(i));
    psi_dot_desired = dpsi_d(t(i));

    q_desired = euler2q(phi_desired, theta_desired, psi_desired);
    q_desired_conj = [q_desired(1);
                      -q_desired(2:4)];

    q_tilde = quatmul(q_desired_conj, q);

    w_desired = Tzyx(phi_desired, theta_desired) \ [0;theta_dot_desired;psi_dot_desired];
    w_tilde = w - w_desired;

    % A \ B = inv(A)*B, A / B = A*inv(B)
    

   % Control law
   
   %tau = 1e-4*[0.5 1 -1]';      
   tau = -eye(3)*kd * w_tilde - kp*q_tilde(2:end);

   [phi,theta,psi] = q2euler(q); % Transform q to Euler angles
   
   % Store data for presentation
   simdata(i,:) = [q' phi theta psi w' tau' q_tilde(2:4)'];  % Store data in table
   
   % State propagation: q[k+1] is computed using the matrix exponential, 
   % which serves as the exponential map for matrix Lie groups, ensuring an 
   % exact discretization of the quaternion differential equation: 
   % q_dot = Tquat(w_imu - b_ars + sigma) * q
   % You can replace the build-in Matlab function expm.m with the custom-made 
   % MSS functions expm_squaresPade.m for this computation
   q = expm( Tquat(w) * h ) * q;    % Exact discretization, do not use Euler’s method on SO(3)   
   q  = q / norm(q);                % Unit quaternion normalization

   % Angular velocity propagation w[k+1] is computed using approximate discretization.
   % Based on accuracy and computation time requirements either a lower order method such as Euler's method 
   % or a higher order method such as Runge-Kutta 4 can be used.
   
   % Euler's method (not recommended)
   % w_dot = I_inv * (Smtrx(Ib * w) * w + tau); % Rigid-body kinetics
   % w = w + h * w_dot;                         % State update

   % Runge-Kutta 4 method
   function_w_dot = @(w, I_inv, Ib, tau) I_inv * (Smtrx(Ib * w) * w + tau);  % Rigid-body kinetics
   w = rk4(function_w_dot, h, w, I_inv, Ib, tau);                            % State update

end 

%% PLOTS 
close all;
q       = simdata(:,1:4); 
phi     = rad2deg(simdata(:,5));
theta   = rad2deg(simdata(:,6));
psi     = rad2deg(simdata(:,7));
w       = rad2deg(simdata(:,8:10));  
tau     = simdata(:,11:13);
q_tilde = simdata(:,14:16);


figure (1); clf;
hold on;
plot(t, phi, 'b');
plot(t, theta, 'r');
plot(t, psi, 'g');
hold off;
grid on;
legend('\phi', '\theta', '\psi');
title('Euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

figure (2); clf;
hold on;
plot(t, w(:,1), 'b');
plot(t, w(:,2), 'r');
plot(t, w(:,3), 'g');
hold off;
grid on;
legend('p', 'q', 'r');
title('Angular velocities');
xlabel('time [s]'); 
ylabel('angular rate [deg/s]');
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

figure (3); clf;
hold on;
plot(t, tau(:,1), 'b');
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

figure (4); clf;
hold on;
plot(t, q_tilde(:,1), 'b');
plot(t, q_tilde(:,2), 'r');
plot(t, q_tilde(:,3), 'g');
hold off;
grid on;
legend('e_1', 'e_2', 'e_3');
title('Error');
xlabel('time [s]'); 
ylabel('error [?]');
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

% Animate the satellite
record = false;
enable_animation = true;
if enable_animation
    animateSatelliteSTL(t, simdata(:,5), simdata(:,6), simdata(:,7), 'satellite.stl', record);
end


%% LOCAL FUNCTIONS

function result = quatmul(q1,q2)
n1 = q1(1);
n2 = q2(1);
e1 = q1(2:4);  % Extract the vector part of the quaternion
e2 = q2(2:4);
result = [n1*n2 - dot(e1,e2); 
    n1*e2 + n2*e1 + cross(e1,e2)];
% Till next time:
%...............................n1*e2 +n2*e1 ...
% Produces a separator [n1*e2, n2*e1] instead of [n1*e2+n2*e1,...]
end