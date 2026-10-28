% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0 0; 0 Yvdot Yrdot; 0 Nvdot Nrdot ];

psi_ref = 10 * pi/180;  % desired yaw angle (rad)
U_ref   = 7;            % desired surge speed (m/s)


% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

% input matrix
t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   


T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
D = -diag([Xu Yv Nr]);



% rigid-body mass matrix
MRB = [ m 0    0 
    0 m    m*xg
    0 m*xg Iz ];
Minv_mini = invQR(MRB(2:3,2:3) + MA(2:3,2:3));


psi_d = psi_ref;
r_d = 0;
u_d = U_ref;

C_comb = [0, (m-Xudot)*u_d;
    (Xudot - Yvdot)*u_d, (m*xg - Yrdot)*u_d];
N = C_comb + D(2:3,2:3);

b = 2*u_d*[-Y_delta; -N_delta];

% [b,a] = ss2tf(A,B,C,D)
tf_A = -Minv_mini*N;
tf_B = Minv_mini*b;
tf_C = [0 1];
tf_D = 0;
[num,den] = ss2tf(tf_A, tf_B, tf_C, tf_D);