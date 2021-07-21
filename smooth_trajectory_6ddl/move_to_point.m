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

  % Polynomials coefficients : s(t) = A4*t^5 + A5*t^5 + A6*t^6 + A7*t^7
  
  A4 = 35;
  A5 = -84;
  A6 = 70;
  A7 = -20; 
  % Position, speed, acceleration and jerk polynomials 
  s    = @(t)    A4*t.^4 +    A5*t.^5  +     A6*t.^6  +     A7*t.^7; % position 
  sp   = @(t)  4*A4*t.^3 +  5*A5*t.^4  +   6*A6*t.^5  +   7*A7*t.^6; % speed 
  spp  = @(t) 12*A4*t.^2 + 20*A5*t.^3  +  30*A6*t.^4  +  42*A7*t.^5; % acceleration 
  sppp = @(t) 24*A4*t   +  60*A5*t.^2  + 120*A6*t.^3  + 210*A7*t.^4; % jerk 
  
  %% Speed, acceleration and jerk min and max
  
  SP_MAX = 35/16;
  
  SPP_MAX =  85*sqrt(5)/25;
  SPP_MIN = -85*sqrt(5)/25;
  
  SPPP_MAX = 42;
  SPPP_MIN = -105/25;

  deltaX = pointB(1) - pointA(1);
  deltaY = pointB(2) - pointA(2);
  deltaZ = pointB(3) - pointA(3);
  
  deltaRoll  = orientationB(1) - orientationA(1); 
  deltaPitch = orientationB(2) - orientationA(2); 
  deltaYaw   = orientationB(3) - orientationA(3); 

  T_min_speed_x = abs(deltaX)*SP_MAX/V_MAX;
  T_min_acc_x   = sqrt(abs(deltaX)*SPP_MAX/A_MAX);

  T_min_speed_y = abs(deltaY)*SP_MAX/V_MAX;
  T_min_acc_y   = sqrt(abs(deltaY)*SPP_MAX/A_MAX);
  
  T_min_speed_z = abs(deltaZ)*SP_MAX/V_MAX;
  T_min_acc_z   = sqrt(abs(deltaZ)*SPP_MAX/A_MAX);
  
  T_min_speed_roll = abs(deltaRoll)*SP_MAX/W_RP;
  T_min_acc_roll   = sqrt(abs(deltaRoll)*SPP_MAX/A_RP);
  
  T_min_speed_pitch = abs(deltaPitch)*SP_MAX/W_RP;
  T_min_acc_pitch   = sqrt(abs(deltaPitch)*SPP_MAX/A_RP);
  
  T_min_speed_yaw = abs(deltaYaw)*SP_MAX/W_YAW;
  T_min_acc_yaw   = sqrt(abs(deltaYaw)*SPP_MAX/A_YAW);

  T = max([T_min_speed_x,T_min_speed_y,T_min_speed_z,...
      T_min_acc_x, T_min_acc_y,T_min_acc_z...
      T_min_speed_roll,T_min_speed_pitch,T_min_speed_yaw,...
      T_min_acc_roll, T_min_acc_pitch,T_min_acc_yaw]); 
  
  t0 = 0:Ts:T;
  t = [ t_in, t0+t_in(end) ];
  t_dot = t0/T; 

  if ( deltaX ~= 0 )
    traj.x   = [traj.x  , pointA(1)+deltaX*s(t_dot)]; 
    traj.x_s = [traj.x_s,       1/T*deltaX*sp(t_dot)];
    traj.x_a = [traj.x_a,  1/(T.^2)*deltaX*spp(t_dot)];
    traj.x_j = [traj.x_j,  1/(T.^3)*deltaX*sppp(t_dot)];
  else
    traj.x_j =  [ traj.x_j , zeros(size(t0))]; % acceleration along the x axis 
    traj.x_a =  [ traj.x_a , zeros(size(t0))]; % acceleration along the x axis 
    traj.x_s =  [ traj.x_s , zeros(size(t0))]; % speed along the x axis 
    traj.x   =  [ traj.x , traj.x(end)*ones(size(t0))]; % trajectory along the x axis
  end
  
 if ( deltaY ~= 0 )
    traj.y   = [traj.y  , pointA(2)+deltaY*s(t_dot)]; 
    traj.y_s = [traj.y_s,       1/T*deltaY*sp(t_dot)];
    traj.y_a = [traj.y_a,  1/(T.^2)*deltaY*spp(t_dot)];
    traj.y_j = [traj.y_j,  1/(T.^3)*deltaY*sppp(t_dot)];
  else
    traj.y_j = [ traj.y_j , zeros(size(t0))]; % acceleration along the y axis 
    traj.y_a = [ traj.y_a , zeros(size(t0))]; % acceleration along the y axis 
    traj.y_s = [ traj.y_s , zeros(size(t0))]; % speed along the y axis 
    traj.y   = [ traj.y , traj.y(end)*ones(size(t0))]; % trajectory along the y axis
 end
 
 if ( deltaZ ~= 0 )
    traj.z   = [traj.z  , pointA(3)+deltaZ*s(t_dot)]; 
    traj.z_s = [traj.z_s,       1/T*deltaZ*sp(t_dot)];
    traj.z_a = [traj.z_a,  1/(T.^2)*deltaZ*spp(t_dot)];
    traj.z_j = [traj.z_j,  1/(T.^3)*deltaZ*sppp(t_dot)];
  else
    traj.z_j = [ traj.z_j , zeros(size(t0))]; % acceleration along the z axis 
    traj.z_a = [ traj.z_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.z_s = [ traj.z_s , zeros(size(t0))]; % speed along the z axis 
    traj.z   = [ traj.z , traj.z(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 if ( deltaRoll ~= 0 )
    traj.roll   = [traj.roll  , orientationA(1)+deltaRoll*s(t_dot)]; 
    traj.roll_s = [traj.roll_s,       1/T*deltaRoll*sp(t_dot)];
    traj.roll_a = [traj.roll_a,  1/(T.^2)*deltaRoll*spp(t_dot)];
    traj.roll_j = [traj.roll_j,  1/(T.^3)*deltaRoll*sppp(t_dot)];
  else
    traj.roll_j = [ traj.roll_j , zeros(size(t0))]; % acceleration along the z axis 
    traj.roll_a = [ traj.roll_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.roll_s = [ traj.roll_s , zeros(size(t0))]; % speed along the z axis 
    traj.roll   = [ traj.roll , traj.roll(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 if ( deltaPitch ~= 0 )
    traj.pitch   = [traj.pitch  , orientationA(2)+deltaPitch*s(t_dot)]; 
    traj.pitch_s = [traj.pitch_s,       1/T*deltaPitch*sp(t_dot)];
    traj.pitch_a = [traj.pitch_a,  1/(T.^2)*deltaPitch*spp(t_dot)];
    traj.pitch_j = [traj.pitch_j,  1/(T.^3)*deltaPitch*sppp(t_dot)];
  else
    traj.pitch_j = [ traj.pitch_j , zeros(size(t0))]; % acceleration along the z axis 
    traj.pitch_a = [ traj.pitch_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.pitch_s = [ traj.pitch_s , zeros(size(t0))]; % speed along the z axis 
    traj.pitch   = [ traj.pitch , traj.pitch(end)*ones(size(t0))]; % trajectory along the z axis
 end
 
 if ( deltaYaw ~= 0 )
    traj.yaw   = [traj.yaw  , orientationA(3)+deltaYaw*s(t_dot)]; 
    traj.yaw_s = [traj.yaw_s,       1/T*deltaYaw*sp(t_dot)];
    traj.yaw_a = [traj.yaw_a,  1/(T.^2)*deltaYaw*spp(t_dot)];
    traj.yaw_j = [traj.yaw_j,  1/(T.^3)*deltaYaw*sppp(t_dot)];
  else
    traj.yaw_j = [ traj.yaw_j , zeros(size(t0))]; % acceleration along the z axis 
    traj.yaw_a = [ traj.yaw_a , zeros(size(t0))]; % acceleration along the z axis 
    traj.yaw_s = [ traj.yaw_s , zeros(size(t0))]; % speed along the z axis 
    traj.yaw   = [ traj.yaw , traj.yaw(end)*ones(size(t0))]; % trajectory along the z axis
 end
end
