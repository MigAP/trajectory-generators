%%% Generates the trajectory to move from point A to B
%%% pointA = [xa; ya]
%%% pointB = [xb; yb]

function [t,traj] = move_to_point(t_in, traj_in, robot,pointA, pointB,orientationA,orientationB)
  traj = traj_in; 
  
  
  %% System Properties
  Ts    = robot.Ts; % [s] sampling time 
  A_MAX = robot.A_MAX; % [m/s^2]
  V_MAX = robot.V_MAX;  %  [m/s]
  % Orientation : 
  A_RP  = robot.A_RP;
  W_RP  = robot.W_RP;
  A_YAW = robot.A_YAW;
  W_YAW = robot.W_YAW;

  Ts_integrator = robot.Ts_integrator; % [s] sampling period 
  %% Speed, acceleration min and max
 
  deltaX = pointB(1) - pointA(1);
  deltaY = pointB(2) - pointA(2);
  deltaZ = pointB(3) - pointA(3);
  
  deltaRoll  = orientationB(1) - orientationA(1); 
  deltaPitch = orientationB(2) - orientationA(2); 
  deltaYaw   = orientationB(3) - orientationA(3); 
  
  % Trajectory duration, acceleration time and deceleration time
  T_x     = get_duration(V_MAX,A_MAX,deltaX); 
  T_acc_pos   = V_MAX/A_MAX; % independent of the position DoF
 
  T_y     = get_duration(V_MAX,A_MAX,deltaY); 
 
  T_z     = get_duration(V_MAX,A_MAX,deltaZ); 
 
  
  T_roll     = get_duration(W_RP,A_RP,deltaRoll); 
  T_acc_rp   = W_RP/A_RP; % independent of the orientation DoF
  
  T_pitch   = get_duration(W_RP,A_RP,deltaPitch); 
    
  T_yaw    = get_duration(W_YAW,A_YAW,deltaYaw); 
  T_acc_yaw   = W_YAW/A_YAW; % independent of the orientation DoF
  
  % Final trajectory duration 
  T = max([T_x,T_y,T_z,...
      T_roll,T_pitch,T_yaw]); 
  
  t0 = 0:Ts_integrator:T;
  t = [ t_in, t0+t_in(end) ];
  
  % Deceleration times 
  T_dec_pos = T - T_acc_pos; 
  T_dec_rp  = T - T_acc_rp; 
  T_dec_yaw = T - T_acc_yaw; 
  
  % Acceleration  indices
  i_acc_pos = find(t0 >= T_acc_pos,1); 
  i_acc_rp  = find(t0 >= T_acc_rp,1); 
  i_acc_yaw = find(t0 >= T_acc_yaw,1); 
  
  %Deceleration indices
  i_dec_pos = find(t0 >= T_dec_pos,1); 
  i_dec_rp  = find(t0 >= T_dec_rp,1); 
  i_dec_yaw = find(t0 >= T_dec_yaw,1); 
  
  if ( deltaX ~= 0 )
    
    acc_profile_x = zeros(size(t0)); 
    acc_profile_x(1:i_acc_pos)     = A_MAX*sign(deltaX); 
    acc_profile_x(i_dec_pos:end)   = -A_MAX*sign(deltaX); 
                    
    speed_profile_x = integrate_vector(traj.x_s(end), acc_profile_x, Ts_integrator);
    
    position_profile_x = integrate_vector(traj.x(end), speed_profile_x, Ts_integrator);
    
    traj.x_a   = [traj.x_a, acc_profile_x]; 
    traj.x_s = [traj.x_s, speed_profile_x ];
    traj.x = [traj.x,  position_profile_x];
    
  else
    traj.x_a =  [ traj.x_a , zeros(size(t0))]; % acceleration along the x axis 
    traj.x_s =  [ traj.x_s , zeros(size(t0))]; % speed along the x axis 
    traj.x   =  [ traj.x , traj.x(end)*ones(size(t0))]; % trajectory along the x axis
  end
  
 if ( deltaY ~= 0 )
     
    acc_profile_y = zeros(size(t0)); 
    acc_profile_y(1:i_acc_pos)     = A_MAX*sign(deltaY); 
    acc_profile_y(i_dec_pos:end)   = -A_MAX*sign(deltaY);
                    
    speed_profile_y = integrate_vector(traj.y_s(end), acc_profile_y, Ts_integrator);
    
    position_profile_y = integrate_vector(traj.y(end), speed_profile_y, Ts_integrator);
    
    traj.y_a   = [traj.y_a, acc_profile_y]; 
    traj.y_s = [traj.y_s, speed_profile_y ];
    traj.y = [traj.y,  position_profile_y];
 else
    traj.y_a = [ traj.y_a , zeros(size(t0))]; % acceleration along the y axis 
    traj.y_s = [ traj.y_s , zeros(size(t0))]; % speed along the y axis 
    traj.y   = [ traj.y , traj.y(end)*ones(size(t0))]; % trajectory along the y axis
 end
 
 if ( deltaZ ~= 0 )
     
    acc_profile_z = zeros(size(t0)); 
    acc_profile_z(1:i_acc_pos)     = A_MAX*sign(deltaZ); 
    acc_profile_z(i_dec_pos:end)   = -A_MAX*sign(deltaZ); 
                    
    speed_profile_z = integrate_vector(traj.z_s(end), acc_profile_z, Ts_integrator);
    
    position_profile_z = integrate_vector(traj.z(end), speed_profile_z, Ts_integrator);
    
    traj.z_a   = [traj.z_a, acc_profile_z]; 
    traj.z_s = [traj.z_s, speed_profile_z ];
    traj.z = [traj.z,  position_profile_z];
 else
    traj.z_a = [ traj.z_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.z_s = [ traj.z_s , zeros(size(t0))]; % speed along the z axis 
    traj.z   = [ traj.z , traj.z(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 
 
 if ( deltaRoll ~= 0 )
     
    acc_profile_roll = zeros(size(t0)); 
    acc_profile_roll(1:i_acc_rp)     = A_RP*sign(deltaRoll); 
    acc_profile_roll(i_dec_rp:end)   = -A_RP*sign(deltaRoll); 
                    
    speed_profile_roll = integrate_vector(traj.roll_s(end), acc_profile_roll, Ts_integrator);
    
    position_profile_roll = integrate_vector(traj.roll(end), speed_profile_roll, Ts_integrator);
    
    traj.roll_a   = [traj.roll_a, acc_profile_roll]; 
    traj.roll_s = [traj.roll_s, speed_profile_roll ];
    traj.roll = [traj.roll,  position_profile_roll];
    
 else
    traj.roll_a = [ traj.roll_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.roll_s = [ traj.roll_s , zeros(size(t0))]; % speed along the z axis 
    traj.roll   = [ traj.roll , traj.roll(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 if ( deltaPitch ~= 0 )
     
    acc_profile_pitch = zeros(size(t0)); 
    acc_profile_pitch(1:i_acc_rp)     = A_RP*sign(deltaPitch); 
    acc_profile_pitch(i_dec_rp:end)   = -A_RP*sign(deltaPitch); 
                    
    speed_profile_pitch = integrate_vector(traj.pitch_s(end), acc_profile_pitch, Ts_integrator);
    
    position_profile_pitch = integrate_vector(traj.pitch(end), speed_profile_pitch, Ts_integrator);
    
    traj.pitch_a   = [traj.pitch_a, acc_profile_pitch]; 
    traj.pitch_s = [traj.pitch_s, speed_profile_pitch ];
    traj.pitch = [traj.pitch,  position_profile_pitch];
 else
    traj.pitch_a = [ traj.pitch_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.pitch_s = [ traj.pitch_s , zeros(size(t0))]; % speed along the z axis 
    traj.pitch   = [ traj.pitch , traj.pitch(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 if ( deltaYaw ~= 0 )
     
    acc_profile_yaw = zeros(size(t0)); 
    acc_profile_yaw(1:i_acc_rp)     = A_YAW*sign(deltaYaw); 
    acc_profile_yaw(i_dec_rp:end)   = -A_YAW*sign(deltaYaw); 
                    
    speed_profile_yaw = integrate_vector(traj.yaw_s(end), acc_profile_yaw, Ts_integrator);
    
    position_profile_yaw = integrate_vector(traj.yaw(end), speed_profile_yaw, Ts_integrator);
    
    traj.yaw_a   = [traj.yaw_a, acc_profile_yaw]; 
    traj.yaw_s = [traj.yaw_s, speed_profile_yaw ];
    traj.yaw = [traj.yaw,  position_profile_yaw];
 else
    traj.yaw_a = [ traj.yaw_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.yaw_s = [ traj.yaw_s , zeros(size(t0))]; % speed along the z axis 
    traj.yaw   = [ traj.yaw , traj.yaw(end)*ones(size(t0))]; % trajectory along the z axis
 end
end
