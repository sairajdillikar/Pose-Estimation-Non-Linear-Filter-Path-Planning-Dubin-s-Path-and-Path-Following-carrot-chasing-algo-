%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%   CCA_Straight.m                          %
%                                           %
%                                           %
%            Created by Late Min-Guk Seo    %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialization
% clear all ;
% close all ;
% clc ;


function [Cur_pos,Cur_Head_ang] = CCA_Straight(En_WP, Rob_Pos, Head_Ang)

%% Parameters
%.. Angle Converting Parameters
r2d                     =               180 / pi ;          % Radian to Degree [-]
d2r                     =               1 / r2d ;           % Degree to Radian [-]

%.. Time Step Size
dt                      =               0.1 ;               % Time Step Size [s]

%.. Time
t(1)                    =               0 ;                 % Simulation Time [s]

%.. Waypoint
Wi                      =               Rob_Pos ;         % Initial Waypoint Position [m]
Wf                      =               En_WP ;     % Final Waypoint Position [m]

%.. Position and Velocity of Robot
x(1)                    =               Rob_Pos(1,1) ;               % Initial Robot X Position [m]
y(1)                    =               Rob_Pos(2,1) ;                 % Initial Robot Y Position [m]
psi(1)                  =               Head_Ang * d2r ;           % Initial Robot Heading Angle [rad]

p(:,1)                  =               [ x(1), y(1) ]' ;   % Robot Position Initialization [m]
va                      =               5 ;                % Robot Velocity [m/s]

%.. Maximum Lateral Acceleration of Robot
Rmin                    =               5 ;                % Robot Minimum Turn Radius [m]
umax                    =               va^2 / Rmin ;      % Robot Maximum Lateral Acceleration [m]

%.. Design Parameters
kappa                   =               20;              % Gain
delta                   =               5000 ;                 % Carrot Distance

%% Path Following Algorithm
i                       =               0 ;                 % Time Index
while abs(En_WP(1,1)-x(i+1))>0.2 && abs(En_WP(2,1)-y(i+1))>0.2
    i                   =               i + 1 ;
    
    %==============================================================================%
    %.. Path Following Algorithm
    
    % Step 1
    % Distance between initial waypoint and current Robot position, Ru
    Ru                  =    norm(Wi - p(:,i),2)            ;    
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta               =    atan2(Wf(2)-Wi(2),Wf(1)-Wi(1))            ;
    
    % Step 2
    % Orientation of vector from initial waypoint to current Robot position, thetau
    thetau              =     atan2((p(2,i)-Wi(2)),(p(1,i)-Wi(1)))           ;
    % Difference between theta and theatu, DEL_theta
    DEL_theta           =     theta - thetau           ;
    
    % Step 3
    % Distance between initial waypoint and q, R
    R                   =      sqrt((Ru)^2-(Ru*sin(DEL_theta))^2)          ;
    
    % Step 4
    % Carrot position, s = ( xt, yt )
    xt                  =       (R+delta)*(cos(theta))         ;
    yt                  =       (R+delta)*(sin(theta))         ;
    
    % Step 5
    % Desired heading angle, psid
    psid                =       atan2((yt-p(2,i)),(xt-p(1,i)))         ;
    % Wrapping up psid
    psid = rem(psid,2*pi);
    if psid < -pi
        psid = psid + 2*pi;
    elseif psid > pi
        psid = psid-2*pi;
    end
    
    % Step6
    % Guidance command, u
    u(i)                =       (kappa*(psid-psi(i))*va);
    % Limit u
    if u(i) > umax
        u(i)            =               umax;
    elseif u(i) < -umax
        u(i)            =             - umax;
    end
    %==============================================================================%
    
    %.. Robot Dynamics
    % Dynamic Model of Robot
    dx                  =               va * cos( psi(i) ) ;
    dy                  =               va * sin( psi(i) ) ;
    dpsi                =               u(i) / va ;
    
    % Robot State Update
    x(i+1)              =               x(i) + dx * dt ;
    y(i+1)              =               y(i) + dy * dt ;
    psi(i+1)            =               psi(i) + dpsi * dt ;
    
    % Robot Position Vector Update
    p(:,i+1)            =               [ x(i+1), y(i+1) ]' ;
    
    %.. Time Update
    t(i+1)              =               t(i) + dt ;
    
    x_pos = x(i+1);
    y_pos = y(i+1);
    Cur_pos = [x_pos;y_pos];
    Cur_Head_ang = psi(i+1);

    if i==500
        break
    end
end


%% Result Plot
%.. Trajectory Plot
figure(1) ;
% plot( [ Wi(1), Wf(1) ], [ Wi(2), Wf(2) ], 'k', 'LineWidth', 1 ) ;
% hold on ;
plot( x, y, 'b', 'LineWidth', 1) ;
hold on ;
% xlabel('X (m)') ;
% ylabel('Y (m)') ;
% legend('Robot Trajectory', 'Desired Path', 'Location', 'southeast' ) ;
% hold on;
% axis([ -15 200 -15 200 ]) ;

% %.. Guidance Command
% figure(2) ;
% plot( t(1:end-1), u, 'LineWidth', 2 ) ;
% xlabel('Time (s)') ;
% ylabel('u (m/s^2)') ;

end

