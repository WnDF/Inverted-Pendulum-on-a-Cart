%% Main Script for Models, Estimators and Controllers
clear;clc;
Ts = 0.01;
h = Ts;
Ttotal = 30;
%% Identified parameters (See invPend_id.m results)
g = 9.81;
alpha = 10;

a1 = 0.362015;
a2 = 113.636338;
k1 = -2000.714368;
k2 = 238.679879;
k3 = 0.009382;
ks = 101.924343;
kd = 15.346870;

%% Linearized Models:

% Select equilibrium for linearization
modelType = 'upward';

% Equilibrium input when nonlinear dynamics are solved w.r.t. fixed points
ue = -kd/k1;

switch lower(modelType)

    case 'downward'

        xe = [0; 0; 0; 0];

        A = [ ...
            0, 0, 1, 0;
            0, 0, 0, 1;
            0,  -g/(a1*a2 - 1), -a1*(k2 + alpha*ks)/(a1*a2 - 1),  k3/(a1*a2 - 1);
            0, a2*g/(a1*a2 - 1),  (k2 + alpha*ks)/(a1*a2 - 1), -a2*k3/(a1*a2 - 1)];

        B = [ ...
            0;
            0;
            a1*k1/(a1*a2 - 1);
           -k1/(a1*a2 - 1)];

    case 'upward'

        xe = [0; pi; 0; 0];

        A = [ ...
            0, 0, 1, 0;
            0, 0, 0, 1;
            0,  -g/(a1*a2 - 1), -a1*(k2 + alpha*ks)/(a1*a2 - 1), -k3/(a1*a2 - 1);
            0,  -a2*g/(a1*a2 - 1), -(k2 + alpha*ks)/(a1*a2 - 1), -a2*k3/(a1*a2 - 1)];

        B = [ ...
            0;
            0;
            a1*k1/(a1*a2 - 1);
            k1/(a1*a2 - 1)];
end

C = [1 0 0 0;
     0 1 0 0];

D = [0;
     0];

% Continuous-time state-space model
sys_c = ss(A, B, C, D);

% Discrete-time state-space model
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

%% LQR Controller
Q_lqr = diag([80 100 75 100]);
R_lqr = 50;
K = dlqr(Ad,Bd,Q_lqr,R_lqr);

%% LQI Controller

% Augmented Model
Ai = [Ad, zeros(4,1);
     -Ts*Cd(1,:), 1];

Bi = [Bd;
      0];

Q_lqi = diag([75 100 75 100 250]);  
R_lqi = 75;

K_aug = dlqr(Ai, Bi, Q_lqi, R_lqi);

Kx = K_aug(1:4);
Ki = K_aug(5);

%% Reference Tracking Signal for LQI

t_ref = (0:Ts:Ttotal)';

A = 0.70;      % position amplitude
f = 0.065;      % frequency

x_ref = A*sin(2*pi*f*t_ref);
x_ref_dot = gradient(x_ref, Ts);

inputType = 'reference';
switch lower(inputType)
    case 'stabilization'
        xref_state_data = [zeros(size(t_ref)),zeros(size(t_ref)), zeros(size(t_ref)), zeros(size(t_ref))];
    case 'reference'
        xref_state_data = [x_ref,zeros(size(t_ref)), x_ref_dot, zeros(size(t_ref))];
end

ref_track = timeseries(xref_state_data, t_ref);
ref_track = setuniformtime(ref_track, ...
    'StartTime', 0, ...
    'Interval', Ts);
% plot(ref_track)

%% MPC
C_mpc = eye(4);
D_mpc = zeros(4,1);

% Define MPC plant
plant_mpc = ss(Ad, Bd, C_mpc, D_mpc, Ts);

% MPC horizons
PredictionHorizon = 40;   % 0.4 s
ControlHorizon    = 10;   % 0.1 s

% Defining MPC object
mpcobj = mpc(plant_mpc, Ts, PredictionHorizon, ControlHorizon);

% Allow own Kalman filter - not using built-in
setEstimator(mpcobj,'custom');
setoutdist(mpcobj,'model',tf(zeros(4,1)))

mpcobj.MV.Min = -0.75;
mpcobj.MV.Max =  0.75;

% Output constraints
mpcobj.OV(1).Min = -0.8;
mpcobj.OV(1).Max =  0.8;

mpcobj.OV(2).Min = -0.2;
mpcobj.OV(2).Max =  0.2;

% Weights
mpcobj.Weights.OV = [95 100 30 10];      
mpcobj.Weights.MV = 12.5;

% Nominal conditions
mpcobj.Model.Nominal.X = [0;0;0;0];
mpcobj.Model.Nominal.U = ue;
mpcobj.Model.Nominal.Y = [0;0];

% Initial MPC state
xmpc = mpcstate(mpcobj);

%% MPC reference tracking trajectory
A = 0.70;      % position amplitude 
f = 0.065;      % frequency 

x_ref = A*sin(2*pi*f*t_ref);
x_dot_ref = gradient(x_ref,Ts);


inputType = 'stabilization';
switch lower(inputType)
    case 'stabilization'
        xref_state_data = [zeros(size(t_ref)),zeros(size(t_ref)), zeros(size(t_ref)), zeros(size(t_ref))];
    case 'reference'
        xref_state_data = [x_ref,zeros(size(t_ref)), x_ref_dot, zeros(size(t_ref))];
end


ref_track_mpc = timeseries(xref_state_data, t_ref);
ref_track_mpc = setuniformtime(ref_track_mpc, ...
    'StartTime', 0, ...
    'Interval', Ts);

% plot(ref_track_mpc)

%% Swing Up Zero Input

swingUpZeros = [zeros(size(t_ref))];
swingUpZero = timeseries(swingUpZeros, t_ref);
swingUpZero = setuniformtime(swingUpZero, ...
    'StartTime', 0, ...
    'Interval', Ts);
% plot(swingUpZero)
