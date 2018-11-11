function percentage = rho(ang)
%RHO The force policy 'rho' of the swing leg describes the reduction 
% percentage [0,1] of the force on the stance leg caused by the swing leg  
% as a function of the angle 'ang' of the stance leg.
%   Detailed explanation goes here

% The coefficients of the cubic spline describing the policy.
global p2;

% Calculate polynomial describing policy 'delta' in 'ang'.
% It replace the polynoom used in earlier versions.
% percentage = polyval(p,ang);
percentage = fnval(p2,ang);

end

