%% Identification Stage 2 - Cart Dynamics Identification
clear; clc; close all;

% Load data
load('fixedPendulumTrainingData.mat');

t_all   = expData.time(:);
pos_all = expData.output.position(:);
u_all   = expData.input.u(:);
Ts      = expData.sampleTime;

% Split data: 75% training, 25% validation
N = length(t_all);
N_train = floor(0.75*N);

idx_train = 1:N_train;
idx_val   = N_train+1:N;

t_train   = t_all(idx_train);
pos_train = pos_all(idx_train);
u_train   = u_all(idx_train);

t_val   = t_all(idx_val);
pos_val = pos_all(idx_val);
u_val   = u_all(idx_val);

% reset time vectors
t_train = t_train - t_train(1);
t_val   = t_val   - t_val(1);

% iddata objects
data_train = iddata(pos_train, u_train, Ts);
data_train.OutputName = {'Cart position'};
data_train.InputName  = {'PWM input'};
data_train.OutputUnit = {'m'};
data_train.TimeUnit   = 'seconds';

data_val = iddata(pos_val, u_val, Ts);
data_val.OutputName = {'Cart position'};
data_val.InputName  = {'PWM input'};
data_val.OutputUnit = {'m'};
data_val.TimeUnit   = 'seconds';

% Ploting training data and validation data
figure;

subplot(2,2,1)
plot(t_train,u_train,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('PWM')
title('Training data: Cart Dynamics Input %75')

subplot(2,2,2)
plot(t_train,pos_train,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
title('Training data: Cart Dynamics Output %75')

subplot(2,2,3)
plot(t_val,u_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('PWM')
title('Validation data: Cart Dynamics Input %25')

subplot(2,2,4)
plot(t_val,pos_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
title('Validation data: Cart Dynamics Output %25')

% Parameters and Initials
m  = 0.872;
p1 = 9.4;
p2 = -0.548;
fc = 0.5;
fs = 1.203;

K1_0 = -p1/m;
K2_0 = (fc - p2)/m;
Kc_0 = fs/m;
d_0  = 0;

Parameters = {K1_0; K2_0; Kc_0; d_0};

% Initial states
pos0 = pos_train(1);

% for smooth derivatives
pos_s = smoothdata(pos_train,'sgolay',21);
vel_s = gradient(pos_s,Ts);
vel0 = vel_s(1);
x0 = [pos0; vel0];

% Grey-box model
Order = [1 1 2];
sys0 = idnlgrey('fixed_pendulum_model', Order, Parameters, x0, 0);

sys0.Parameters(1).Name = 'K1';
sys0.Parameters(2).Name = 'K2';
sys0.Parameters(3).Name = 'Kc';
sys0.Parameters(4).Name = 'd';

% Parameters bounds
sys0.Parameters(1).Minimum = -100;
sys0.Parameters(1).Maximum = 100;

sys0.Parameters(2).Minimum = 0;
sys0.Parameters(2).Maximum = 20;

sys0.Parameters(3).Minimum = 0;
sys0.Parameters(3).Maximum = 20;

sys0.Parameters(4).Minimum = -10;
sys0.Parameters(4).Maximum = 10;

% Setting initials for ID
sys0.InitialStates(1).Name = 'position';
sys0.InitialStates(2).Name = 'velocity';
sys0.InitialStates(1).Fixed = false;
sys0.InitialStates(2).Fixed = false;

% ID Options
opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchMethod = 'auto';

% Identification
sys_cart_est = nlgreyest(data_train, sys0, opt);

% Results
K1_hat = sys_cart_est.Parameters(1).Value;
K2_hat = sys_cart_est.Parameters(2).Value;
Kc_hat = sys_cart_est.Parameters(3).Value;
d_hat  = sys_cart_est.Parameters(4).Value;

fprintf('\nEstimated parameters:\n');
fprintf('K1_hat = %.6f\n', K1_hat);
fprintf('K2_hat = %.6f\n', K2_hat);
fprintf('Kc_hat = %.6f\n', Kc_hat);
fprintf('d_hat  = %.6f\n', d_hat);

% Converted full-model cart-side parameters
l = 0.011;
a2_0 = 1/l;

k1_hat = a2_0*K1_hat;
k2_hat = a2_0*K2_hat;
ks_hat = a2_0*Kc_hat;
kd_hat = a2_0*d_hat;

fprintf('\nConverted full-model parameters:\n');
fprintf('k1_hat = %.6f\n', k1_hat);
fprintf('k2_hat = %.6f\n', k2_hat);
fprintf('ks_hat = %.6f\n', ks_hat);
fprintf('kd_hat = %.6f\n', kd_hat);

% Identification Results
compareOpt = compareOptions;
compareOpt.InitialCondition = 'estimate';
figure;
compare(data_train, sys_cart_est, compareOpt);
title('Training fit: Cart Dynamics');

% Identification - Model Validation
figure;
compare(data_val, sys_cart_est, compareOpt);
title('Validation fit: Cart Dynamics');