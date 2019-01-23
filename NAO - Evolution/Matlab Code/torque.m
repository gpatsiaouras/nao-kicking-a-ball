function tor = torque( length, force )
%TORQUE This function detemines the torque on the knee joint. It is a
% function of the angle of the knee joint and the force on the leg.
% Although there should always hold 'delta(.)<=1', because of inaccuracies
% in the generation of the polynomial, 'delta(.)' may be larger than 1.
% These inaccuracies are not visible in the first 4 digits behind the dot,
% but they do cause problems. For this reason the max-function is used.

global l;

%tor = 0.5 * l * sqrt(max(1 - delta(ang).^2, 0)) .* force;

% This version contains a correction for the distance between the ankle
% joint and the hip joint of the Nao, which is around 20 cm 
% while l is 28.5 cm.
corrected_l = 0.2;

tor = 0.5 * sqrt(max(corrected_l^2 - (corrected_l+length-l).^2, 0)) .* force;
end

