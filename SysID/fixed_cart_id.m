%% Identification Stage 1 - Pendulum Dynamics Identification 
clear; clc; close all;

% Load data
load('fixedCartTrainingData.mat');

t_all     = expData.time(:);
theta_all = expData.output.angle(:);
Ts        = expData.sampleTime;

% Split data: 75% training, 25% validation
N = length(theta_all);
N_train = floor(0.75*N);

idx_train = 1:N_train;
idx_val   = N_train+1:N;

t_train = t_all(idx_train);
theta_train = theta_all(idx_train);

t_val = t_all(idx_val);
theta_val = theta_all(idx_val);

% reset time vectors
t_train = t_train - t_train(1);
t_val   = t_val   - t_val(1);

% Dummy inputs (data holders)
u_train = zeros(length(theta_train),1);
u_val   = zeros(length(theta_val),1);

% iddata objects
data_train = iddata(theta_train, u_train, Ts);
data_train.OutputName = {'Pendulum angle'};
data_train.InputName  = {'Dummy input'};
data_train.OutputUnit = {'rad'};
data_train.TimeUnit   = 'seconds';

data_val = iddata(theta_val, u_val, Ts);
data_val.OutputName = {'Pendulum angle'};
data_val.InputName  = {'Dummy input'};
data_val.OutputUnit = {'rad'};
data_val.TimeUnit   = 'seconds';

% Ploting training data and validation data
figure;

subplot(2,1,1)
plot(t_train,theta_train,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('\theta [rad]')
title('Training data: Pendulum Dynamics %75')

subplot(2,1,2)
plot(t_val,theta_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('\theta [rad]')
title('Validation data: Pendulum Dynamics %25')

% Parameters and Initials
m  = 0.872;
l  = 0.011;
Jp = 0.00292;
fp = 6.65e-5;

a1_0 = Jp/(m*l);
k3_0 = fp/(m*l);

Parameters = {a1_0; k3_0};

% Initial states
theta0 = theta_train(1);

% for smooth derivatives
theta_s = smoothdata(theta_train,'sgolay',21);
theta_dot_s = gradient(theta_s,Ts);
theta_dot0 = theta_dot_s(1);
x0 = [theta0; theta_dot0];

% Grey-box model
Order = [1 1 2];
sys0 = idnlgrey('fixed_cart_model', Order, Parameters, x0, 0);

sys0.Parameters(1).Name = 'a1';
sys0.Parameters(2).Name = 'k3';

% Parameters bounds
sys0.Parameters(1).Minimum = 0.20;
sys0.Parameters(1).Maximum = 0.45;

sys0.Parameters(2).Minimum = 1e-5;
sys0.Parameters(2).Maximum = 0.02;

% Setting initials for ID
sys0.InitialStates(1).Name = 'theta';
sys0.InitialStates(2).Name = 'theta_dot';
sys0.InitialStates(1).Fixed = false;
sys0.InitialStates(2).Fixed = false;

% ID Options
opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchMethod = 'lsqnonlin';

% Identification 
sys_pend_est = nlgreyest(data_train, sys0, opt);

% Results
a1_hat = sys_pend_est.Parameters(1).Value;
k3_hat = sys_pend_est.Parameters(2).Value;

fprintf('\nEstimated parameters:\n');
fprintf('a1_hat = %.6f\n', a1_hat);
fprintf('k3_hat = %.6f\n', k3_hat);

% Identification Results
compareOpt = compareOptions;
compareOpt.InitialCondition = 'estimate';
figure;
compare(data_train, sys_pend_est, compareOpt);
title('Training fit: Pendulum Dynamics');

% Identification - Model Validation
figure;
compare(data_val, sys_pend_est, compareOpt);
title('Validation fit: Pendulum Dynamics');