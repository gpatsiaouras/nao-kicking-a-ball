function force = Gforce( t, angle, omega )
%GFORCE This function calculates the redial force on the stance leg caused  
% by the gravity and the changes in the length of the leg. The force is  
% determined by the second derivative of the changes in the leg length.
% Note that the moment the leg (foot) hits the ground, because of the fixed
% leg length there might be a discontinuity of the speed in the radial 
% direction of the new stance leg. This dicontinuity results in a peak in
% the force. Although we need to consider this too, this wil NOT be done
% here but in the function 'Rforce'.

% constants
global m;	% the mass of the Nao
global g;   % gravitational acceleration
global l;   % maximum (abstract) leg length; max height center of mass
global b;   % the friction constant; friction linear in the speed

% Calculate the acceleration 'Domega' of the angle.
Domega = -(Ddelta(angle) .* omega.^2) ./ delta(angle) - b/m * omega + (g * sin(angle)) ./ (delta(angle) * l);

% Calculate the redial force.
force = m * g * cos(angle) + m * l * (DDdelta(angle) .* omega.^2 + Ddelta(angle) .* Domega);
%force = m * l * (DDdelta(angle) .* omega.^2 + Ddelta(angle) .* Domega);

end

