%% closedLoop script for testing the F16 lateral autopilot systems

clearvars; 

% n = 0: open-loop tests
% n = 1: roll attitude and sideslip hold
% n = 2: course and sideslip hold
n = 2;

%% Simulator parameters
D2R = pi / 180;     % degrees to radians
R2D = 180 / pi;     % radians to degrees
g = 9.81;           % acceleration of gravity [m/s^2]

T_final = 200;	    % Final simulation time (s)
h = 0.01;           % Sampling time (s)

% Aircraft model: x = [beta p r phi psi]' and u = [ delta_a delta_r ]'
A = [ -0.1203 0.1479 -0.9860 0.0636 0
      -17.5184 -1.2523 0.5484 0 0
       3.5318 -0.0274 -0.1788 0 0
       0 1 0.1489 0 0 
       0 0 1 0 0];
 
B = [ 0.0001  0.0003 
     -0.2560 -0.0044
     -0.0133 -0.0292
      0 0
      0 0 ];

a_phi1  = -A(2,2);         % roll and sideslip transfer functions 
a_phi2  =  B(2,1);         % p_dot = -a_phi1 * p + a_phi2 * delta_a
a_beta1 = -A(1,1);         % beta_dot = -a_beta1 * beta + a_beta2 * delta_r
a_beta2 =  B(1,2);

delta_a_max = 21 * D2R;   % maximum aileron [rad]
delta_r_max = 30 * D2R;   % maximum rudder [rad]

Vg = 257.9;               % ground speed [m/s]

%% MAIN LOOP
x = [0 0 0 0 0]';	                        % initial states and commands
phi_int = 0; chi_int = 0; beta_int = 0;
phi_c = 0; chi_c = 0;

t = 0:h:T_final;             % Time vector
nTimeSteps = length(t);      % Number of time steps

simdata = zeros(nTimeSteps, 13); % Pre-allocate matrix for efficiency

for i = 1:nTimeSteps
    
	% Measurements: y[k]
	beta = x(1); p = x(2); r = x(3); phi = x(4); psi = x(5);
    chi = psi;       % assumes that beta = 0

	% Autopilot test cases
    if n == 0       

        delta_a_c = 25 * sin(t(i)/10) * D2R;        
        delta_r_c = -40 * sin(t(i)/10) * D2R; 

    elseif n == 1   % roll attitude and sideslip hold

        if t(i) < 30
            phi_ref = 0; 
        elseif t(i) >= 30 && t(i) < 50
            phi_ref = 15 * D2R;
            phi_c = exp(-h/5) * phi_c + (1 - exp(-h/5)) * phi_ref; 
        elseif t(i) >= 50 && t(i) < 150
            phi_ref = 5 * D2R;
            phi_c = exp(-h/5) * phi_c + (1 - exp(-h/5)) * phi_ref;             
       else
           phi_c = exp(-h/5) * phi_c; 
       end
       
       % ****************** CALL TO YOUR FUNCTIONS ************************
       [delta_a_c,phi_int] = rollAttitudeHold(a_phi1,a_phi2,p,phi,phi_c,phi_int,h);
       [delta_r_c,beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h);
       % ******************************************************************
     
    else  % course and sideslip hold

        if t(i) <= 30
            chi_c = 0;
        elseif t(i) > 30 && t(i) < 60
            chi_ref = 10 * D2R;   
            chi_c = exp(-h/5) * chi_c + (1 - exp(-h/5)) * chi_ref;
        else
            chi_c = exp(-h/5) * chi_c; 
        end          

       % ****************** CALL TO YOUR FUNCTIONS ************************
        [phi_c,chi_int] = courseHold(chi,chi_c,Vg,chi_int,h);
        [delta_a_c,phi_int] = rollAttitudeHold(a_phi1,a_phi2,p,phi,phi_c,phi_int,h);
        [delta_r_c,beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h);
       % ******************************************************************        

    end

    % Control signals
    delta_a = delta_a_c;
    if abs(delta_a_c) > delta_a_max                % aileron saturation
        delta_a = sign(delta_a_c) * delta_a_max;
    end

    delta_r = delta_r_c;
    if abs(delta_r_c) > delta_r_max                % rudder saturation
        delta_r = sign(delta_r_c) * delta_r_max;
    end
    u = [ delta_a delta_r ]';

    % Store simulation data in a data table   
    simdata(i,:) = [t(i) psi chi beta phi p r...
        delta_a delta_a_c delta_r delta_r_c phi_c chi_c]; 

	% Propagate states to x[k+1] by Euler's method
    d  = -0.1176 * [ A(1,1) A(2,1) A(3,1) 0 0]' * D2R;    % disturbance 
	x = x + h * ( A * x + B * u + d );                    % aircraft model

end

%% Plot simulation data
plotResults(simdata,n);     


%% Animate aircraft STL using simulation data
stlFile = "F16.stl";
animateFlight = true;           % Enable animation
record = false;                 % Record the animation
third_person_view = false;      % Enable third person view of plane
show_body_axes = false;         % Display the body axis of the plane
show_yaw_vector = false;        % Display yaw vector direction
view_scale = 1;                 % Scale the animation
animationStep = 100;            % Number of simulation samples between displayed frames.
playbackSpeed = 30;             % Playback speed relative to real simulation time.

if animateFlight
    videoFile = "F16_lateral_autopilot.mp4";
    animateAircraftSTL(simdata, stlFile, Vg, record, videoFile, third_person_view, view_scale, animationStep, playbackSpeed, show_body_axes, show_yaw_vector);
end