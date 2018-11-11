function Ds = gait(t, s)
%GAIT This function specifies the first and second order derivative over
% time of the leg angle w.r.t. the vertical line. 
% 't' is the time variable and 's' the state variable. The first element in
% 's' descrbes the current angle 'ang' and the the second element the
% angle speed 'angSpeed'.


% constants
global m;	% the mass of the Nao
global g;   % gravitational acceleration
global l;   % maximum (abstract) leg length; max height center of mass
global b;   % the friction constant; friction linear in the speed

% variables
ang  = s(1); 
angSpeed = s(2);


% second order differential equations describing one step
% 'F(x)' is the force policy of the stance leg of the Nao
DangSpeed = -(Ddelta(ang) * angSpeed^2) / delta(ang) - b/m * angSpeed + (g * sin(ang)) / (delta(ang) * l);
%DangSpeed = (g * sin(ang)) / l;
Dang = angSpeed;

% result
Ds = [ Dang; DangSpeed ];

end

