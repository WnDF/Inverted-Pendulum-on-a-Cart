function [dx, y] = fixed_cart_model(~, x, u, a1, k3, varargin)
% Fixed-cart pendulum model

% States:
% x1 = pendulum angle
% x2 = pendulum angular velocity

% Input:
% u = dummy input 

% Parameters:
% a1 = inertia-related coefficient
% k3 = viscous damping coefficient

% Output:
% y = pendulum angle

g = 9.81;

theta     = x(1);
theta_dot = x(2);
dx = zeros(2,1);
dx(1) = theta_dot;
dx(2) = -(g/a1)*sin(theta) - (k3/a1)*theta_dot;
y = theta;

end