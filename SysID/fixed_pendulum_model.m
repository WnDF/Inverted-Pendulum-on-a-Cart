function [dx, y] = fixed_pendulum_model(~, x, u, K1, K2, Kc, d, varargin)
% Fixed-pendulum cart model

% States:
% x1 = cart position
% x2 = cart velocity

% Input:
% u = PWM input

% Parameters:
% K1 = input gain
% K2 = viscous damping
% Kc = Coulomb/static friction gain
% d  = constant acceleration bias 

alpha = 10;

pos = x(1);
vel = x(2);
dx = zeros(2,1);
dx(1) = vel;
dx(2) = K1*u - K2*vel - Kc*tanh(alpha*vel) + d;

y = pos;

end