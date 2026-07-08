%% Identification Stage 3 - Coupled Cart-Pendulum Dynamics Identification
clear; clc; close all;

% Load data
load('coupledTrainingData.mat');

t_all     = expData.time(:);
u_all     = expData.input.u(:);
pos_all   = expData.output.position(:);
angle_all = expData.output.angle(:);
Ts        = expData.sampleTime;

% Angle preprocessing
angle_mean = mean(angle_all);
angle_all = angle_all - angle_mean;

% Split data: 75% training, 25% validation
N = length(t_all);
N_train = floor(0.75*N);

idx_train = 1:N_train;
idx_val   = N_train+1:N;

t_train     = t_all(idx_train);
u_train     = u_all(idx_train);
pos_train   = pos_all(idx_train);
angle_train = angle_all(idx_train);

t_val     = t_all(idx_val);
u_val     = u_all(idx_val);
pos_val   = pos_all(idx_val);
angle_val = angle_all(idx_val);

% reset time vectors
t_train = t_train - t_train(1);
t_val   = t_val   - t_val(1);

% iddata objects
y_train = [pos_train angle_train];
y_val   = [pos_val angle_val];

data_train = iddata(y_train, u_train, Ts);
data_train.OutputName = {'Cart position','Pendulum angle'};
data_train.InputName  = {'PWM input'};
data_train.OutputUnit = {'m','rad'};
data_train.TimeUnit   = 'seconds';

data_val = iddata(y_val, u_val, Ts);
data_val.OutputName = {'Cart position','Pendulum angle'};
data_val.InputName  = {'PWM input'};
data_val.OutputUnit = {'m','rad'};
data_val.TimeUnit   = 'seconds';

% Ploting training data and validation data
figure;

subplot(2,3,1)
plot(t_train,u_train,'LineWidth',1.2)
grid on
ylabel('PWM')
title('Training data: Coupled Dynamics Input %75')

subplot(2,3,2)
plot(t_train,pos_train,'LineWidth',1.2)
grid on
ylabel('Position [m]')
title('Training data: Coupled Dynamics Position %75')

subplot(2,3,3)
plot(t_train,angle_train,'LineWidth',1.2)
grid on
ylabel('Angle [rad]')
title('Training data: Coupled Dynamics Angle %75')

subplot(2,3,4)
plot(t_val,u_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('PWM')
title('Validation data: Coupled Dynamics Input %25')

subplot(2,3,5)
plot(t_val,pos_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
title('Validation data: Coupled Dynamics Position %25')

subplot(2,3,6)
plot(t_val,angle_val,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
title('Validation data: Coupled Dynamics Angle %25')

% Parameters and Initials

% Identified pendulum dynamics parameters
a1_0 = 0.367691;
k3_0 = 0.012509;

% Identified cart dynamics parameters
K1_hat = -22.709508;
K2_hat =  2.155194;
Kc_hat =  1.494818;
d_hat  =  0.176016;

% Geometry
l = 0.011;
a2_0 = 1/l;

% Convert reduced cart parameters to full-model parameters
k1_0 = a2_0*K1_hat;
k2_0 = a2_0*K2_hat;
ks_0 = a2_0*Kc_hat;
kd_0 = a2_0*d_hat;

Parameters = {a1_0; a2_0; k1_0; k2_0; k3_0; ks_0; kd_0};

% Initial states
pos0   = pos_train(1);
angle0 = angle_train(1);

% for smooth derivatives
pos_s = smoothdata(pos_train,'sgolay',21);
ang_s = smoothdata(angle_train,'sgolay',21);

vel_s    = gradient(pos_s,Ts);
angvel_s = gradient(ang_s,Ts);

vel0    = vel_s(1);
angvel0 = angvel_s(1);

x0 = [pos0; angle0; vel0; angvel0];

% Grey-box model
Order = [2 1 4];
sys0 = idnlgrey('invPend_model', Order, Parameters, x0, 0);

names = {'a1','a2','k1','k2','k3','ks','kd'};

for i = 1:numel(names)
    sys0.Parameters(i).Name = names{i};
end

% Parameters bounds
sys0.Parameters(1).Minimum = 0.75*a1_0;
sys0.Parameters(1).Maximum = 1.25*a1_0;

sys0.Parameters(2).Minimum = 0.75*a2_0;
sys0.Parameters(2).Maximum = 1.25*a2_0;

% k1_0 is negative
sys0.Parameters(3).Minimum = 1.25*k1_0;
sys0.Parameters(3).Maximum = 0.75*k1_0;

sys0.Parameters(4).Minimum = 0.75*k2_0;
sys0.Parameters(4).Maximum = 1.25*k2_0;

sys0.Parameters(5).Minimum = 0.75*k3_0;
sys0.Parameters(5).Maximum = 1.25*k3_0;

sys0.Parameters(6).Minimum = 0.75*ks_0;
sys0.Parameters(6).Maximum = 1.25*ks_0;

sys0.Parameters(7).Minimum = 0.75*kd_0;
sys0.Parameters(7).Maximum = 1.25*kd_0;

% Setting initials for ID
stateNames = {'position','angle','velocity','angular_velocity'};

for i = 1:4
    sys0.InitialStates(i).Name = stateNames{i};
    sys0.InitialStates(i).Fixed = false;
end

sys0.InitialStates(1).Value = pos0;
sys0.InitialStates(2).Value = angle0;
sys0.InitialStates(3).Value = vel0;
sys0.InitialStates(4).Value = angvel0;

% Free/fix parameters
for i = 1:7
    sys0.Parameters(i).Fixed = false;
end

% a2 is fixed because it is defined by geometry as a2 = 1/l
sys0.Parameters(2).Fixed = true;

% ID Options
opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchMethod = 'lsqnonlin';

% Identification
sys_full_est = nlgreyest(data_train, sys0, opt);

% Results
a1_hat = sys_full_est.Parameters(1).Value;
a2_hat = sys_full_est.Parameters(2).Value;
k1_hat = sys_full_est.Parameters(3).Value;
k2_hat = sys_full_est.Parameters(4).Value;
k3_hat = sys_full_est.Parameters(5).Value;
ks_hat = sys_full_est.Parameters(6).Value;
kd_hat = sys_full_est.Parameters(7).Value;

fprintf('\nEstimated parameters:\n');
fprintf('a1_hat = %.6f\n', a1_hat);
fprintf('a2_hat = %.6f\n', a2_hat);
fprintf('k1_hat = %.6f\n', k1_hat);
fprintf('k2_hat = %.6f\n', k2_hat);
fprintf('k3_hat = %.6f\n', k3_hat);
fprintf('ks_hat = %.6f\n', ks_hat);
fprintf('kd_hat = %.6f\n', kd_hat);

% Identification Results
compareOpt = compareOptions;
compareOpt.InitialCondition = 'estimate';
figure;
compare(data_train, sys_full_est, compareOpt);
title('Training fit: Coupled Cart-Pendulum Dynamics');

% Identification - Model Validation
figure;
compare(data_val, sys_full_est, compareOpt);
title('Validation fit: Coupled Cart-Pendulum Dynamics');