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
