function percentage = delta(ang)
%DELTA The leg length policy 'delta' describes the percentage in [0,1] of
% shortening the leg length as a function of the 'ang'.
%   Detailed explanation goes here

% The coefficients of the cubic spline describing the policy.
global p;

% Calculate polynomial describing policy 'delta' in 'ang'.
% It replace the polynoom used in earlier versions.
% percentage = polyval(p,ang);
percentage = fnval(p,ang);

end

