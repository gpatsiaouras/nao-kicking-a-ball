function [ lookfor stop direction ] = leg_change( t, s )
%LEG_CHANGE Stopping criteria for the differential equation solver.
% 's' is the state vector.

% The maximal angle.
global angE;

% 'step' is the current step size of the robot
lookfor = s(1) - angE;	% the angle 'ang' is maximal
stop = 1;           % stop when event occurs
direction = 1;      % forward motion 

end

