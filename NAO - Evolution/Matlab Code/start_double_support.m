function [ lookfor stop direction ] = start_double_support( t, s )
%START_DOUBLE_SUPPORT Stopping criteria for the differential equation solver.
% 's' is the state vector.

% The maximal angle.
global angD;

% 'step' is the current step size of the robot
lookfor = s(1) - angD;	% the angle 'ang' is maximal
stop = 1;           % stop when event occurs
direction = 1;      % forward motion 

end

