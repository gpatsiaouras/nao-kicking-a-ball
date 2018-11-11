% Simulator of the gait of a Nao robot with a double support phase
% given a leg length policy 'delta(ang)' of  the stance leg and 
% a force policy '' of the swing leg in the double support phase.

clear *;
warning('off','all');

% The maximum length of a leg in the abstract model. The maximum height of
% the center of mass.
global l;
l = 0.285;

% The mass 'm' of the robot.
global m;
m = 4.996;   	% the mass of the Nao

% The graviational constant.
global g;
g = 9.80665;    % gravitational acceleration

% The step size.
global s;
s = 0.08;

% The friction constant; friction linear in the speed.
global b;
b = 1;

% The intial speed.
spB = 0.9;

% Cubic spline coeficients. It replaces the polynomial for describing leg 
% length policy. The generated polynoom caused instabilities known as the 
% "Runge's phenomenon".
global p;

% Cubic spline coeficients describing the force policy.
global p2;

% The maximal angles.
global angB;
global angE;
global angD; % angle at which the double support phase is starting

% The percentage of the step forming the double support phase.
double = 0.25;

% The leg length policy is detemined by the relative length of 
% the leg at 'n' points during a step 'rl[1:n]'.
n = 9;
max = 4;

% The impact time of the foot hitting the ground at the beginning of a new
% step.
it = 0.01;

% Set initial leg length policy.
rlmax = 1;   % rlmax <= 1
rlmin = 0.998;   % rlmin > 1 - s/l
rlstep = 0.001;
for i = 1:n
    rl(i) = rlmax; 
end;

% rl(1) = 1;
% rl(2) = 0.85;
% rl(3) = 1;
% rl(4) = 1;
% rl(5) = 1;
% rl(6) = 1;
% rl(7) = 1;
% rl(8) = 1;
% rl(9) = 1;

% Initialize energy consumption.
energy = 1000000;

% for testing
c1 = 0;
%global c3;
c3 = 0;
% Determine a polynomial for for the leg length policy using these values, 
% do a simulation, and generate new values for 'rl[1:n]'. Repeat the
% process for all possible policies.
ready = false;
while ~ready
    c1 = c1 + 1;
    c3 = c3 + 1;
    
    % The values 'rl(1)' and 'rl(n)' determine the maxiamal angle 'ang' 
    % at the beginning an the end of a step respectively. We denote the 
    % maxima by 'angB' and 'angE' respecively. We use the cosin-rule to
    % determine these angles.
    % Note that we use 'asin' instead of 'acos' becuase we need the angle
    % with the vertical axis instead of the horizontal axis.
    angB = -asin( (s^2 + (l*rl(1))^2 - (l*rl(n))^2) / (2 * s * l*rl(1)) );
    angE = asin( (s^2 + (l*rl(n))^2 - (l*rl(1))^2) / (2 * s * l*rl(n)) );
    angD = angE - double*(angE-angB);
    
    angS = (angE - angB) / (n-1);
    
    % Determine the cubic spline 'delta(ang)' describing the leg policy.
    % It replace the polynoom used in earlier versions.
    angI = [angB:angS:angE];

    % p = polyfit(angI,rl,n-1);
    p = csapi(angI,rl);
    
    % Set initial state. The minimal angle speed when legs are complete
    % stretched at the time is '0.826'. Below this numebr the highest
    % point will not be reached. We choose to an speed of '0.9'.
%    s_init = [angB; 0.9]; % the initial angle and angle speed
    s_init = [angB; spB]; % the initial angle and angle speed
    
    % Set stop event
    options = odeset('Events', @start_double_support);
    
    % Simulate half a step. The result is the angle and angle speed at
    % different time points (coloms 1 an 2 in Y, respectively).
    [T, Y, TE, YE, IE, sol]  = ode45(@gait, [0,1], s_init, options);
    % 'T' time points
    % 'Y' solution a each time point
    % 'TE' time at which event occurred
    % 'YE' solution at time of event
    % 'IE' index of event
    % 'sol' structure to evaluate the solution

    % Detemine the initial speed allong the path of the mass 'm'.
    % 'x' and 'z' form the Cartesian coordinatie system.
    vB_t = l * Y(1,2) * delta(angB);
    vB_r = l * Y(1,2) * Ddelta(angB);
    vB_x = vB_t * cos(angB) + vB_r * sin(angB);
    vB_z = vB_t * sin(angB) + vB_r * cos(angB);
    vB = sqrt(vB_x^2 + vB_z^2);

    % Detemine the final speed of the single support phase along the 
    % path of the mass 'm'.
    % 'x' and 'z' form the Cartesian coordinatie system.
    vD_t = l * Y(length(Y(:,2)),2) * delta(angD);
    vD_r = l * Y(length(Y(:,2)),2) * Ddelta(angD);
    vD_x = vD_t * cos(angD) + vD_r * sin(angD);
    vD_z = vD_t * sin(angD) + vD_r * cos(angD);
    vD = sqrt(vD_x^2 + vD_z^2);
    
    % The angle speed at the end of the single support phase.
    spD = Y(length(Y(:,2)),2);

    % Determine whether the leg length policy is successful.
    if ~isempty(IE) && (IE(1) == 1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate the double support phase.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % The force policy of the swing leg in the double support phase is 
        % detemined by a fraction of the gravitational force of the stance leg
        % at 'n2' points during a the double support phase 'fp[1:n]'.
        n2 = 2;

        % Set initial force policy.
        fpmax = 1;
        fpmin = 0;
        fpstep = 0.5;
        for i = 1:n2
            fp(i) = fpmax; 
        end;

        % fp(1) = 0;
        % fp(2) = 0.25;
        % fp(3) = 0.5;
        % fp(4) = 0.75;
        % fp(5) = 1;

        c2 = 0;

        % Determine a polynomial for for the leg length policy using these values, 
        % do a simulation, and generate new values for 'fp[1:n]'. Repeat the
        % process for all possible policies.
        ready2 = false;
        while ~ready2
            c2 = c2 + 1;
            c3 = c3 + 1;

            % Determine the cubic spline 'rho(ang)' describing the force policy.
            angS2 = (angE - angD) / (n2-1);
            angI2 = [angD:angS2:angE];

            p2 = csapi(angI2,fp);

            % Set initial state. The angle speed at the start of the double support
            % phase is equal to the angle speed at the end of the single support phase.
            s_init2 = [angD; spD]; % the initial angle and angle speed

            % Set stop event
            options2 = odeset('Events', @leg_change);

            % Simulate half a step.
            [T2, Y2, TE2, YE2, IE2, sol2]  = ode45(@gait_double, [0,1], s_init2, options2);
                % 'T' time points
                % 'Y' solution a each time point
                % 'TE' time at which event occurred
                % 'YE' solution at time of event
                % 'IE' index of event
                % 'sol' structure to evaluate the solution

            % Detemine the initial speed allong the path of the mass 'm' at the
            % begining of the double support phase.
            % 'x' and 'z' form the Cartesian coordinatie system.
            vD_t = l * Y2(1,2) * delta(angD);
            vD_r = l * Y2(1,2) * Ddelta(angD);
            vD_x = vD_t * cos(angD) + vD_r * sin(angD);
            vD_z = vD_t * sin(angD) + vD_r * cos(angD);
            vD = sqrt(vD_x^2 + vD_z^2);

            % Detemine the final speed allong the path of the mass 'm'.
            % 'x' and 'z' form the Cartesian coordinatie system.
            vE_t = l * Y2(length(Y2(:,2)),2) * delta(angE);
            vE_r = l * Y2(length(Y2(:,2)),2) * Ddelta(angE);
            vE_x = vE_t * cos(angE) + vE_r * sin(angE);
            vE_z = vE_t * sin(angE) + vE_r * cos(angE);
            vE = sqrt(vE_x^2 + vE_z^2);

            % Calculate the impulse on the swing leg when it becomes the new stance 
            % leg at the end of the double support phase. 
            % The calculation uses the difference between the component of 'vE'
            % in the direction of the new stance leg, and 'vB_r'.
            % This is approximation is only correct if 'vB=vE', which should
            % hold if the energy consumption is minimal.
            imp = m * (vB_r - vE_x*sin(angB) - vE_z*cos(angB));

            % Determine whether the leg length and force policy are successful.
            if ~isempty(IE2) && (IE2(1) == 1) && (imp >= 0) && (vE >= vB)         

                % The corresponding impact force.
                iF = imp / it;

%                 % The angel of the swing leg.
%                 h = l * delta(Y2(:,1)) .* cos(Y2(:,1));
%                 s1 = l * delta(Y2(:,1)) .* sin(Y2(:,1));
%                 ang_sw = atan((s-s1) ./ h);
                
                l_st = delta(Y2(:,1))*l;
                l_sw = sqrt(s^2 + l_st.^2 - 2*s*l_st.*sin(Y2(:,1)));
                ang_sw = asin((s^2 + l_sw.^2 - l_st.^2) ./ (2*s*l_sw));

                % The total force on the stance leg in the double support phase.
                TFst = Gforce( T2, Y2(:,1), Y2(:,2) ) .* ( 1-rho(Y2(:,1)) );

                % The total force on the swing leg in the double support phase.
                TFsw = Gforce( T2, Y2(:,1), Y2(:,2) ) .* rho(Y2(:,1)) ./ cos(Y2(:,1)+ang_sw); 

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Total force + impulse force of single support phase 
                TF = Rforce(T,Y(:,1),Y(:,2),imp,it); 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Negative force cannot be generated by a leg. No pull towards the
                % ground. Policies generating negative forces must therefore be
                % ignored.
                % Also note that it avoids negative energy consuption.
                if isempty(find(TF<0))
                    % The energy comsuption of the knee joints.
                    energy_single = trapz(T, torque(delta(Y(:,1))*l,TF) );
                    energy_st =  trapz(T2, torque(delta(Y2(:,1))*l,TFst) );
                    energy_sw =  trapz(T2, torque(l_sw,TFsw) );

                    new_energy = energy_single + energy_st + energy_sw;

                    if new_energy < energy
                        energy = new_energy;
                        impulse = imp;
                        time = T;
                        movement = Y;
                        angle = angB:(angE-angB)/100:angE;
                        policyDef = rl;
                        policy = delta(angle);
                        Dpolicy = Ddelta(angle);
                        DDpolicy = DDdelta(angle);
                        grav_force = Gforce(time,movement(:,1),movement(:,2));
                        tot_force = TF;
                        v_begin = vB;
                        v_end = vE;
                        v_begin_r = vB_r;
                        v_end_x = vE_x;
                        v_end_z = vE_z;
                        v_end_B_r = vE_x*sin(angB) + vE_z*cos(angB);
                        average_speed = s /(TE + TE2);
                        
                        time2 = T2;
                        movement2 = Y2;
                        policyDef2 = fp;
                        E_single = energy_single;
                        E_double_st = energy_st;
                        E_double_sw = energy_sw;
                        angle_begin = angB;
                        angle_double = angD;
                        angle_end = angE;
                        
                        angles_swing_leg = ang_sw;
                        forces_stance_leg = TFst;
                        forces_swing_leg = TFsw;
                        
                    end;
                end;
            end;

        % Stop double support simulation!
        %    ready2 = true;

            % Update force policy.
            next_point2 = true;
            i = 1;
            while ~ready2 && next_point2
                % Update the policy
                fp(i) = fp(i) - fpstep;

                if (fp(i) < fpmin)
                    fp(i) = fpmax;
                    i = i + 1;
                else
                    next_point2 = false;
                end;
                if i > n2
                    ready2 = true;
                end;
            end;
        end;


    % End of double support simulation if singele support simulation was
    % successful.
    end;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% Stop the simulation!
%    ready = true;

    % Update leg length policy.
    next_point = true;
    i = 1;
    % Always start a step with a stretched leg.
    %i = 2;

    while ~ready && next_point
        % Update the policy
        rl(i) = rl(i) - rlstep;
        
        % The values 'rl(1)' and 'rl(n)' determine the maximal angle 'ang' 
        % at the beginning an the end of a step respectively. We denote the 
        % maxima by 'angB' and 'angE' respecively. We use the cosin-rule to
        % determine these angles.
        % Note that we use 'asin' instead of 'acos' because we need the angle
        % with the vertical axis instead of the horizontal axis.
        angB = -asin( (s^2 + (l*rl(1))^2 - (l*rl(n))^2) / (2 * s * l*rl(1)) );
        angE = asin( (s^2 + (l*rl(n))^2 - (l*rl(1))^2) / (2 * s * l*rl(n)) );
        
        if (angB >= 0) || (angE <= 0) || (rl(i) < rlmin) || ~valid(n,rl,angI)
            rl(i) = rlmax;
            i = i + 1;
        else
            next_point = false;
        end;
        if i > max % i > n
            ready = true;
        end;
    end;
end;

if energy == 1000000
    disp('No results');
else
    % Plot angles over time.
    subplot(3,2,1);
    plot(time,movement(:,1), 'k-');
    xlim([0,time(length(time))]);
    xlabel('The angle of the leg over time.');

    % Plot angle speed over time.
    subplot(3,2,2);
    plot(time,movement(:,2), 'k-');
    xlim([0,time(length(time))]);
    xlabel('The angle-speed of the leg over time.');

    % Plot the gravitaional force on the leg.
    subplot(3,2,3);
    plot(time, grav_force, 'k-');
    xlim([0,time(length(time))]);
    xlabel('The redial force on the mass in the over time.');

    % Plot the total force on the leg, including the force caused by the impulse.
    subplot(3,2,4);
    plot(time, tot_force, 'k-');
    xlim([0,time(length(time))]);
    xlabel('The total force including impulse force over time.');

    % Plot policy.
    subplot(3,2,5);
    plot(angle, min(policy,1), 'k-');
    axis tight;
    xlabel('The leg policy over the angle.');

    % Plot policy.
    subplot(3,2,6);
    plot(angle, Dpolicy, 'k-');
    axis tight;
    xlabel('The derivative of the leg policy over the angle.');
    
    disp('Energy:');
    disp(energy);
    
end;





