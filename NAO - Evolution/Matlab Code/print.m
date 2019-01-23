if energy == 1000000
    disp('No results');
else
    % Plot angles over time.
    subplot(3,2,1);
    plot(time,movement(:,1));
    xlim([0,time(length(time))]);
    xlabel('The angle of the leg over time.');

    % Plot angle speed over time.
    subplot(3,2,2);
    plot(time,movement(:,2));
    xlim([0,time(length(time))]);
    xlabel('The angle-speed of the leg over time.');

    % Plot the gravitaional force on the leg.
    subplot(3,2,3);
    plot(time, grav_force);
    xlim([0,time(length(time))]);
    xlabel('The redial force on the mass in the over time.');

    % Plot the total force on the leg, including the force caused by the impulse.
    subplot(3,2,4);
    plot(time, tot_force);
    xlim([0,time(length(time))]);
    xlabel('The total force including impulse force over time.');

    % Plot policy.
    subplot(3,2,5);
    plot(angle, policy);
    axis tight;
    xlabel('The leg policy over the angle.');

    % Plot policy.
    subplot(3,2,6);
    plot(angle, Dpolicy, 'r-');
    axis tight;
    xlabel('The derivative of the leg policy over the angle.');
    
    disp('Energy:');
    disp(energy);
    
end;
