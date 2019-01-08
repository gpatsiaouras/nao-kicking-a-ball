function Ds = gait_double( t, state )
%GAIT_DOUBLE This function specifies the first and second order derivative
% over time of the leg angle w.r.t. the vertical line. 
% 't' is the time variable and 's' the state variable. The first element in
% 'state' describes the current angle 'ang' and the the second element the
% angle speed 'angSpeed'.


% constants
global m;	% the mass of the Nao
global g;   % gravitational acceleration
global l;   % maximum (abstract) leg length; max height center of mass
global b;   % the friction constant; friction linear in the speed
global s;   % step size

% variables
ang  = state(1); 
angSpeed = state(2);

% Angle of swing leg.
l_st = delta(ang)*l;
l_sw = sqrt(s^2 + l_st.^2 - 2*s*l_st.*sin(ang));
ang_sw = asin((s^2 + l_sw.^2 - l_st.^2) ./ (2*s*l_sw));

% Double support break force.
F_b = m*g * sin(ang) * rho(ang) * tan(ang + ang_sw);

% second order differential equations describing one step
% 'F(x)' is the force policy of the stance leg of the Nao
DangSpeed = -(Ddelta(ang) * angSpeed^2) / delta(ang) - b/m * angSpeed + (g * sin(ang) - F_b/m) / (delta(ang) * l);
%DangSpeed = (g * sin(ang)) / l;
Dang = angSpeed;

% result
Ds = [ Dang; DangSpeed ];

end

