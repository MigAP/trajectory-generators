%%% Returns the duration of a trajectory 

function duration = get_duration(v_max,a_max,deltaDistance)

deltaDistance = abs(deltaDistance); 
if deltaDistance > (v_max^2)/a_max
    duration = v_max/a_max + deltaDistance/v_max; 
else
    duration = 2*sqrt(deltaDistance/a_max); 
end
