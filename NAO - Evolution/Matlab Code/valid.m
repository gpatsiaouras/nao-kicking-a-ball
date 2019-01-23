function ok = valid(n,rl,angI)
%VALID Check whether the current state of the leg policy is valid.
% The prejection of mass 'm' on the gound must be moving forward; i.e., 
% for every index of 'rl', '(prev_x < x) && (x < next_x)'. Because of 
% changing the length of the leg this need not be the case. 

global l;

ok = true;
for i=1:n
    if i == 1 
        prev_x = -l;
    else
        prev_x =  l* rl(i-1) * sin(angI(i-1));
    end;
    x =  l* rl(i) * sin(angI(i));
    if i == n
        next_x = l;
    else
        next_x = l* rl(i+1) * sin(angI(i+1));
    end;
    if (prev_x >= x) || (x >= next_x)
        ok = false;
    end;
end

