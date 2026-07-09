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