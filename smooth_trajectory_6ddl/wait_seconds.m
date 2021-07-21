%%% Make the robots wait for s seconds 

function [t,traj] = wait_seconds(t_in, traj_in, Ts,s)

t0 = 0:Ts:s; % trajectory time only for this part 
t = [ t_in, t0+t_in(end) ]; 

traj = traj_in; 

traj.x_j = [ traj.x_j , zeros(size(t0))]; % acceleration along the x axis 
traj.x_a = [ traj.x_a , zeros(size(t0))]; % acceleration along the x axis 
traj.x_s = [ traj.x_s , zeros(size(t0))]; % speed along the x axis 
traj.x=  [ traj.x , traj.x(end)*ones(size(t0))]; % trajectory along the x axis

traj.y_j = [ traj.y_j , zeros(size(t0))]; % acceleration along the y axis 
traj.y_a = [ traj.y_a , zeros(size(t0))]; % acceleration along the y axis 
traj.y_s = [ traj.y_s , zeros(size(t0))]; % speed along the y axis 
traj.y=  [ traj.y , traj.y(end)*ones(size(t0))]; % trajectory along the y axis

traj.z_j = [ traj.z_j , zeros(size(t0))]; % acceleration along the z axis 
traj.z_a = [ traj.z_a , zeros(size(t0))]; % acceleration along the z axis 
traj.z_s = [ traj.z_s , zeros(size(t0))]; % speed along the z axis 
traj.z   = [ traj.z , traj.z(end)*ones(size(t0))]; % trajectory along the z axis

% Orientation 

traj.roll_j = [ traj.roll_j , zeros(size(t0))]; 
traj.roll_a = [ traj.roll_a , zeros(size(t0))]; 
traj.roll_s = [ traj.roll_s , zeros(size(t0))]; 
traj.roll   = [ traj.roll , traj.roll(end)*ones(size(t0))]; 

traj.pitch_j = [ traj.pitch_j , zeros(size(t0))]; 
traj.pitch_a = [ traj.pitch_a , zeros(size(t0))]; 
traj.pitch_s = [ traj.pitch_s , zeros(size(t0))]; 
traj.pitch   = [ traj.pitch , traj.pitch(end)*ones(size(t0))]; 

traj.yaw_j = [ traj.yaw_j , zeros(size(t0))]; 
traj.yaw_a = [ traj.yaw_a , zeros(size(t0))]; 
traj.yaw_s = [ traj.yaw_s , zeros(size(t0))]; 
traj.yaw   = [ traj.yaw , traj.yaw(end)*ones(size(t0))]; 

end
