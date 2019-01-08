function der = Ddelta(ang)
%DDELTA The derivative of the leg length policy 'delta', which describes the 
% percentage in [0,1] of shortening the leg length as a function of the 'ang'.
%   Detailed explanation goes here

% The coefficients of the cubic spline describing the policy.
global p;

% Determine the derivative of the cubic spline described by 'p'.
q = fnder(p);

% Calculate the derivative in 'ang'. 
der = fnval(q,ang);

end

