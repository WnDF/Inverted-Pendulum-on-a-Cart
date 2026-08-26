%% LQR vs MPC Swing-Up Tracking Comparison
clear; clc; close all;

%% Load position data
lqrData = load('lqrPositionCompSwingUp.mat');
mpcData = load('mpcPositionCompSwingUp.mat');

lqrName = fieldnames(lqrData);
mpcName = fieldnames(mpcData);

lqr = lqrData.(lqrName{1});
mpc = mpcData.(mpcName{1});

t_lqr = lqr.time;
t_mpc = mpc.time;

pos_ref = squeeze(lqr.signals(2).values);
pos_lqr = squeeze(lqr.signals(1).values);
pos_mpc = squeeze(mpc.signals(2).values);

%% Load angle data
lqrData = load('lqrAngleCompSwingUp.mat');
mpcData = load('mpcAngleCompSwingUp.mat');

lqrName = fieldnames(lqrData);
mpcName = fieldnames(mpcData);

lqr = lqrData.(lqrName{1});
mpc = mpcData.(mpcName{1});

t_ang_lqr = lqr.time;
t_ang_mpc = mpc.time;

ang_ref = squeeze(lqr.signals(2).values);
ang_lqr = squeeze(lqr.signals(1).values);
ang_mpc = squeeze(mpc.signals(2).values);

%% Figure 1 - Position comparison
figure;
plot(t_lqr,pos_ref,'k--','LineWidth',1.4); hold on
plot(t_lqr,pos_lqr,'g','LineWidth',1.3)
plot(t_mpc,pos_mpc,'LineWidth',1.3)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
title('Position Swing-Up Tracking Comparison')
legend('Reference','LQR','MPC','Location','best')

%% Figure 2 - Angle comparison
figure;
plot(t_ang_lqr,ang_ref,'k--','LineWidth',1.4); hold on
plot(t_ang_lqr,ang_lqr,'g','LineWidth',1.3)
plot(t_ang_mpc,ang_mpc,'LineWidth',1.3)
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
title('Angle Swing-Up Tracking Comparison')
legend('Reference','LQR','MPC','Location','best')