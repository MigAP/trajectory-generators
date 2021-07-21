%%% integrates a vector using specified sample time and initial condition 

function intVec = integrate_vector(x0,vec,Ts)
intVec= cumtrapz(Ts,vec) + x0*ones(size(vec)); 
end
