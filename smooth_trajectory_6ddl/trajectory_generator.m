%%% Trajectory generator using degree 7 polynomial interpolation
%%% Square trajectory generation for the dextair : 
%%% Key poinTs_integrator of the trajectory :  
%%% Peg in the hole: 0->1->2->1->0
%%% Draw: 0->3->4->5->3->0

clear all;
close all;
TRAJ_PLOTS=1; 
SAVE_FIGURE=0; 
%% The point zero is the orign of the fixed frame 
squareLength = 0.2;
squareHeight = 0.2; 
initialHeight = -2;
% Peg in the hole
point0 = [0;0;initialHeight];
point1 = [squareLength      ;squareLength   ;initialHeight]; 


ampYaw = 1*pi/2; 
ampRP = deg2rad(10); 

orientation0 = [0;0;0]; 
orientation1 = [ 0.6424    ;0.1261  ; -0.1419];
orientation2 = [ 0.7617    ;0.0125   ;-0.0949];

%% System Properties
robot.Ts = 0.01; % [s] sampling time 
robot.J_MAX = 3;    % [m/s^3]
robot.A_MAX = 2; % [m/s^2]
robot.V_MAX = 0.5;  %  [m/s]
%Orientation : 
robot.A_RP  = 50;   % [rad/s^2]
robot.W_RP  = pi;   % [rad/s]
robot.A_YAW = 50;   % [rad/s^2]
robot.W_YAW = 2*pi; % [rad/s]

Ts    = robot.Ts; % [s] sampling time 

%% Trajectory generation  
WAIT_TIME = 1; % [s] time that the robots waits in the corners
INIT_WAIT = 2; % [s] initialisation time 


% Initialisation : just wait... 
t = 0:Ts:INIT_WAIT; % trajectory time

traj.x_j = zeros(size(t)); % acceleration along the x axis 
traj.x_a = zeros(size(t)); % acceleration along the x axis 
traj.x_s = zeros(size(t)); % speed along the x axis 
traj.x   = zeros(size(t)); % trajectory along the x axis 

traj.y_j = zeros(size(traj.x_a)); 
traj.y_a = zeros(size(traj.x_a)); 
traj.y_s = zeros(size(traj.x_s)); 
traj.y   = zeros(size(traj.x)); 

traj.z_j = zeros(size(traj.x_a)); 
traj.z_a = zeros(size(traj.x_a)); 
traj.z_s = zeros(size(traj.x_s)); 
traj.z   = ones(size(traj.x))*initialHeight; 

traj.roll_j = zeros(size(t)); 
traj.roll_a = zeros(size(t));
traj.roll_s = zeros(size(t));  
traj.roll   = zeros(size(t));  

traj.pitch_j = zeros(size(t)); 
traj.pitch_a = zeros(size(t));
traj.pitch_s = zeros(size(t));  
traj.pitch   = zeros(size(t));  

traj.yaw_j = zeros(size(t)); 
traj.yaw_a = zeros(size(t));
traj.yaw_s = zeros(size(t));  
traj.yaw   = zeros(size(t));  

%% Peg in the hole: 0->1->2->1->0
[t,traj] = move_to_point(t, traj, robot,point0,point1,orientation0, orientation1);
[t,traj] = wait_seconds(t, traj, Ts,WAIT_TIME);

% [t,traj] = move_to_point(t, traj, robot,point1,point2,orientation1, orientation2);
% [t,traj] = wait_seconds(t, traj, Ts,WAIT_TIME);
% 
% [t,traj] = move_to_point(t, traj, robot,point2,point1,orientation2, orientation1);
% [t,traj] = wait_seconds(t, traj, Ts,WAIT_TIME);

%[t,traj] = move_to_point(t, traj, robot,point1,point0,orientation1, orientation0);
%[t,traj] = wait_seconds(t, traj, Ts,WAIT_TIME);




%% Plots 
if TRAJ_PLOTS 
    plot3(traj.x, traj.y,traj.z);
    title('y en fonction de x'); 
    xlabel('X');
    ylabel('Y'); 
    zlabel('Z');

    figure(); 
    subplot(4,1,1); 
    plot(t,traj.x_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.x_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.x_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.x);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,  axe : x' ]);


    figure(); 
    subplot(4,1,1); 
    plot(t,traj.y_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.y_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.y_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.y);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : y' ]);
    
    figure(); 
    subplot(4,1,1); 
    plot(t,traj.z_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.z_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.z_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.z);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : z' ]);
    
    figure(); 
    subplot(4,1,1); 
    plot(t,traj.roll_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.roll_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.roll_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.roll);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : roll' ]);
    
    figure(); 
    subplot(4,1,1); 
    plot(t,traj.pitch_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.pitch_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.pitch_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.pitch);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : pitch' ]);
    
    figure(); 
    subplot(4,1,1); 
    plot(t,traj.yaw_j)
    title('Jerk'); 
    subplot(4,1,2); 
    plot(t,traj.yaw_a)
    title('Acceleration'); 
    subplot(4,1,3); 
    plot(t,traj.yaw_s)
    title('Speed'); 
    subplot(4,1,4); 
    plot(t,traj.yaw);
    title('Position'); 
    sgtitle(['Acceleration vitesse et position en fonction du temps,axe : yaw' ]);
end 

if SAVE_FIGURE
    figure(); 
    subplot(4,1,1); 
    plot(t,traj.x_j)
    title('Jerk'); 
    xlabel('t [s]'); 
    ylabel('j [$m.s^{-3}$]'); 

    subplot(4,1,2); 
    plot(t,traj.x_a)
    title('Accélération'); 
    xlabel('t [s]'); 
    ylabel('a [$m.s^{-2}$]'); 

    subplot(4,1,3); 
    plot(t,traj.x_s)
    title('Vitesse'); 
    xlabel('t [s]'); 
    ylabel('v [$m.s^{-1}$]'); 

    subplot(4,1,4); 
    plot(t,traj.x);
    title('Position'); 
    xlabel('t [s]'); 
    ylabel('x [$m$]'); 


    print -dpdflatexstandalone -landscape smoothX.pdf
    system('pdflatex smoothX')

    system('rm *.log *-inc.pdf *.tex *.aux'); 

end
