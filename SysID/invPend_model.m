function [dx, y] = invPend_model(~, x, u, a1, a2, k1, k2, k3, ks, kd, varargin)
% Full nonlinear cart-pendulum model

% States:
% x1 = cart position
% x2 = pendulum angle
% x3 = cart velocity
% x4 = pendulum angular velocity

% Input:
% u = PWM input

% Parameters:
% a1 = inertia-related coefficient
% a2 = 1/l
% k1 = input gain
% k2 = cart viscous damping coefficient
% k3 = pendulum viscous damping coefficient
% ks = Coulomb/static friction gain
% kd = constant acceleration bias

% Output:
% y = [cart position; pendulum angle]

g = 9.81;
alpha = 10;

x1 = x(1);   
x2 = x(2);  
x3 = x(3);   
x4 = x(4);   

dx = zeros(4,1);
dx(1) = x3;
dx(2) = x4;
dx(3) = ( ...
    a1*(k1*u ...
    - x4^2*sin(x2) ...
    - k2*x3 ...
    - ks*tanh(alpha*x3) ...
    + kd) ...
    - (g*sin(x2) + k3*x4)*cos(x2)) ...
    / (a1*a2 - cos(x2)^2);

dx(4) = ( ...
    (k1*u ...
    - x4^2*sin(x2) ...
    - k2*x3 ...
    - ks*tanh(alpha*x3) ...
    + kd)*cos(x2) ...
    - a2*(g*sin(x2) + k3*x4)) ...
    / (a1*a2 - cos(x2)^2);

y = [x1; x2];

end

