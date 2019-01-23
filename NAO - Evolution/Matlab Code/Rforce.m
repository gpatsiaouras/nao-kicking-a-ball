function force = Rforce( t, angle, omega, impulse, it )
%RFORCE This function calculates the redial force on the leg caused by the 
% gravity. The force is determined by the second derivative of the changes 
% in the leg length.
% Note that the moment the leg (foot) hits the ground, because of the fixed
% leg length there might be a discontinuity of the speed in the radial 
% direction of the new stance leg. This dicontinuity results in a peak in
% the force, which has to be considered too.

% We assume here that the step starts at t=0. Adapt this function if a
% different start time is used.

force = (t <= it) * (impulse /it) + Gforce( t, angle, omega );
% MATLAB implementation of fi-then-else on vectors

end

