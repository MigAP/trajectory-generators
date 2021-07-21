%%% Square trajectory generation for the dextair : 
%%% Key poinTs_integrator of the trajectory :  
%%%  2------1
%%%  |      |
%%%  |   0  |
%%%  3----- 4
%%% The point zero is the orign of the fixed frame 

clear all; 
close all; 
SAVE_FIGURE=0; 
TRAJ_PLOTS=1; 

%% The point zero is the orign of the fixed frame 
squareLength = 0.2;
initialHeight = -2;

% Peg in the hole
point0 = [0;0;initialHeight];
point1 = [squareLength      ;squareLength   ;initialHeight]; 
point2 = [-squareLength     ;squareLength   ;initialHeight]; 
point3 = [-squareLength     ;-squareLength  ;initialHeight]; 
point4 = [squareLength      ;-squareLength  ;initialHeight]; 

ampYaw = 1*pi/2; 
ampRP = deg2rad(10); 

orientation0 = [0;0;0]; 
orientation1 = [ 0.6424    ;0.1261  ; -0.1419];
orientation2 = [ 0.7617    ;0.0125   ;-0.0949];
%% System Properties

robot.Fs_integrator = 10000; % [Hz] sampling frequency 
robot.Ts_integrator = 1/robot.Fs_integrator; % [s] sampling period 

initialHeight  = -2; 

robot.Ts = 0.01; % [s] sampling time 
robot.A_MAX = 2; % [m/s^2]
robot.V_MAX = 0.5;  %  [m/s]
%Orientation : 
robot.A_RP  = 50;   % [rad/s^2]
robot.W_RP  = pi;   % [rad/s]
robot.A_YAW = 50;   % [rad/s^2]
robot.W_YAW = 2*pi; % [rad/s]

v_max = robot.A_MAX; 
a_max = robot.V_MAX; 

%% Trajectory initialisation

WAIT_TIME = 0.1; % [s] time that the robots waits in the corners
INIT_WAIT = 0; % [s] initialisation time 


% Initialisation : just wait... 
t = 0:robot.Ts:INIT_WAIT; % trajectory time

traj.x_a = zeros(size(t)); % acceleration along the x axis 
traj.x_s = zeros(size(t)); % speed along the x axis 
traj.x   = zeros(size(t)); % trajectory along the x axis 


traj.y_a = zeros(size(traj.x_a)); 
traj.y_s = zeros(size(traj.x_s)); 
traj.y   = zeros(size(traj.x)); 


traj.z_a = zeros(size(traj.x_a)); 
traj.z_s = zeros(size(traj.x_s)); 
traj.z   = ones(size(traj.x))*initialHeight; 


traj.roll_a = zeros(size(t));
traj.roll_s = zeros(size(t));  
traj.roll   = zeros(size(t));  

traj.pitch_a = zeros(size(t));
traj.pitch_s = zeros(size(t));  
traj.pitch   = zeros(size(t));  

traj.yaw_a = zeros(size(t));
traj.yaw_s = zeros(size(t));  
traj.yaw   = zeros(size(t));  

%% Trajetory generation


[t,traj] = move_to_point(t, traj, robot,point0,point1,orientation0, orientation0);
[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);

%[t,traj] = move_to_point(t, traj, robot,point1,point2,orientation0, orientation0);
%[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);
%
%[t,traj] = move_to_point(t, traj, robot,point2,point3,orientation0, orientation0);
%[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);
%
%[t,traj] = move_to_point(t, traj, robot,point3,point4,orientation0, orientation0);
%[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);
%
%[t,traj] = move_to_point(t, traj, robot,point4,point1,orientation0, orientation0);
%[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);
%
%[t,traj] = move_to_point(t, traj, robot,point1,point0,orientation0, orientation0);
%[t,traj] = wait_seconds(t, traj, robot.Ts,WAIT_TIME);

%% Robot's trajectory 

%Fs = 100; %[Hz] sampling frequency of the robot's system
%Ts = 1/Fs; % [s] 
%
%ratio = robot.Fs_integrator/Fs; 
%
%t = t(1:ratio:end); 
%
%traj.x = traj.x(1:ratio:end); 
%traj.x_s = traj.x_s(1:ratio:end); 
%traj.x_a = traj.x_a(1:ratio:end); 
%
%traj.y = traj.y(1:ratio:end); 
%traj.y_s = traj.y_s(1:ratio:end); 
%traj.y_a = traj.y_a(1:ratio:end); 
%
%traj.z = traj.z(1:ratio:end); 
%traj.z_s = traj.z_s(1:ratio:end); 
%traj.z_a = traj.z_a(1:ratio:end); 
%
%traj.roll = traj.roll(1:ratio:end); 
%traj.roll_s = traj.roll_s(1:ratio:end); 
%traj.roll_a = traj.roll_a(1:ratio:end); 
%
%traj.pitch = traj.pitch(1:ratio:end); 
%traj.pitch_s = traj.pitch_s(1:ratio:end); 
%traj.pitch_a = traj.pitch_a(1:ratio:end); 
%
%traj.yaw = traj.yaw(1:ratio:end); 
%traj.yaw_s = traj.yaw_s(1:ratio:end); 
%traj.yaw_a = traj.yaw_a(1:ratio:end); 

%% Check trajectory 

if TRAJ_PLOTS 
    plot(traj.x, traj.y);
    title('y en fonction de x'); 

    figure(); 
    subplot(3,1,1); 
    plot(t,traj.x_a)
    subplot(3,1,2); 
    plot(t,traj.x_s)
    subplot(3,1,3); 
    plot(t,traj.x);
    sgtitle(['Acceleration vitesse et position en fonction du temps,  axe : x' ]);


    figure(); 
    subplot(3,1,1); 
    plot(t,traj.y_a)
    subplot(3,1,2); 
    plot(t,traj.y_s)
    subplot(3,1,3); 
    plot(t,traj.y);
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : y' ]);
    
    figure(); 
    subplot(3,1,1); 
    plot(t,traj.z_a)
    subplot(3,1,2); 
    plot(t,traj.z_s)
    subplot(3,1,3); 
    plot(t,traj.z);
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : z' ]);
    
    figure(); 
    subplot(3,1,1); 
    plot(t,traj.roll_a)
    subplot(3,1,2); 
    plot(t,traj.roll_s)
    subplot(3,1,3); 
    plot(t,traj.roll);
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : roll' ]);
    
    figure(); 
    subplot(3,1,1); 
    plot(t,traj.pitch_a)
    subplot(3,1,2); 
    plot(t,traj.pitch_s)
    subplot(3,1,3); 
    plot(t,traj.pitch);
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : pitch' ]);
    
    figure(); 
    subplot(3,1,1); 
    plot(t,traj.yaw_a)
    subplot(3,1,2); 
    plot(t,traj.yaw_s)
    subplot(3,1,3); 
    plot(t,traj.yaw);
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : roll' ]);
end 

if SAVE_FIGURE
  fig_x =figure(); 
  subplot(3,1,1); 
  plot(t,traj.x_a)
  title('Accélération')
  xlabel('t [s]'); 
  ylabel('a [$m.s^{-2}$]'); 

  subplot(3,1,2); 
  plot(t,traj.x_s)
  title('Vitesse')
  xlabel('t [s]'); 
  ylabel('v [$m.s^{-1}$]'); 

  subplot(3,1,3); 
  plot(t,traj.x);
  title('Position')
  xlabel('t [s]'); 
  ylabel('x [$m$]'); 

  figure(fig_x)
  print -dpdflatexstandalone trapezeX.pdf
  system('pdflatex trapezeX')

  system('rm *.log *-inc.pdf *.tex *.aux'); 

end

%% Debug 

% figure(); 
% plot(t,traj.x,t,traj.y,'b')
% sgtitle(['Position axe x et y en fonction du temps' ]);

