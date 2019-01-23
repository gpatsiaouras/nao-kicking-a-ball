function dder = DDdelta( ang )
%DDDELTA The second order derivative of the leg length policy 'delta', 
% which describes the percentage in [0,1] of shortening the leg length as 
% a function of the 'ang'.
%   Detailed explanation goes here

% The coefficients of the cubic spline describing the policy.
global p;

% Determine the second order derivative of the cubic spline described 
% by 'p'.
q = fnder(p);
r = fnder(q);

% Calculate the second derivative in 'ang'. 
dder = fnval(r,ang);

end

