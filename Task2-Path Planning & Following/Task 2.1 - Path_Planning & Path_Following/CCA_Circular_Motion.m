%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%   CCA_Circular_Motion.m                   %
%                                           %
%                                           %
%            Created by Late Min-Guk Seo    %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialization
% clear all ;
% close all ;
% clc ;

function [Cur_pos,Cur_Head_ang] = CCA_Circular_Motion(Circ_Centre, Fin_WP, Rob_Pos, Head_Ang,lamb,fg,tol)


%% Parameters
%.. Angle Converting Parameters
r2d                     =               180 / pi ;          % Radian to Degree [-]
d2r                     =               1 / r2d ;           % Degree to Radian [-]

%.. Time Step Size
dt                      =               0.1 ;               % Time Step Size [s]

%.. Time
t(1)                    =               0 ;                 % Simulation Time [s]

%.. Circular Orbit
O                       =        Circ_Centre ;         % Centre of Orbit [m]
r                       =               5 ;               % Radius of Orbit [m]

%.. Position and Velocity of Robot
x(1)                    =               Rob_Pos(1,1) ;               % Initial Robot X Position [m]
y(1)                    =               Rob_Pos(2,1) ;                 % Initial Robot Y Position [m]
psi(1)                  =               Head_Ang * d2r ;           % Initial Robot Heading Angle [rad]

p(:,1)                  =               [ x(1), y(1) ]' ;   % Robot Position Initialization [m]
va                      =               5 ;                % Robot Velocity [m/s]

%.. Maximum Lateral Acceleration of Robot
Rmin                    =               5 ;                % Robot Minimum Turn Radius [m]
umax                    =               va^2 / Rmin ;       % Robot Maximum Lateral Acceleration [m]

%.. Design Parameters
kappa                   =               10 ;              % Gain
lambda                  =               fg * lamb * d2r ;          % Carrot Distance      


%% Path Following Algorithm
i                       =               0 ;                 % Time Index
while (abs(Fin_WP(1,1)-x(i+1))>tol && abs(Fin_WP(2,1)-y(i+1))>tol) %== (sqrt( (Fin_WP(1,1)-x(i+1))^2 + (Fin_WP(2,1)-y(i+1))^2 ) > 3 )
    
    i                   =               i + 1 ;
    
    %==============================================================================%
    %.. Path Following Algorithm
    
    % Step 1
    % Distance between orbit and current Robot position, d
    d                   =        norm(O-p(:,1),2)-r        ;
    
    % Step 2
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta               =      atan2(p(2,i)-O(2),(p(1,i)-O(1)))          ;
    
    % Step 3
    % Carrot position, s = ( xt, yt )
    xt                  =      O(1)+(r*(cos(theta+lambda)))          ;
    yt                  =      O(2)+(r*(sin(theta+lambda)))          ;
    
    % Step 4
    % Desired heading angle, psid
    psid                =      atan2((yt-p(2,i)),(xt-p(1,i)))          ;

    % Heading angle error, DEL_psi
    DEL_psi             =      psid-psi(i)          ;
    % Wrapping up DEL_psi
    DEL_psi             =               rem(DEL_psi,2*pi);
    if DEL_psi < -pi
        DEL_psi = DEL_psi + 2*pi;
    elseif DEL_psi > pi
        DEL_psi = DEL_psi-2*pi;
    end
    
    % Step 5
    % Guidance command, u
    u(i)                =      (kappa*(DEL_psi)*va)          ;
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
    Cur_Head_ang = psi(i+1)*r2d;


    if i>500
        break
    end
end




%% Result Plot
% Parameterized Circular Orbit
% TH                      =               0:0.01:2*pi ;
% Xc                      =               O(1) + r * cos( TH ) ;
% Yc                      =               O(2) + r * sin( TH ) ;

%.. Trajectory Plot
figure(1) ;
% plot( Xc, Yc, 'o', 'LineWidth', 1 ) ;
% hold on ;
plot( x, y, 'r', 'LineWidth', 1 ) ;
hold on ;
xlabel('X (m)') ;
ylabel('Y (m)') ;
% title(['Lambda:'num2str(lambdat) ' Kappa= ' num2str(kappa)]);
title(sprintf('Lambda: %g, Kappa: %g, Tolerance: %g', lamb, kappa, tol));
 
% hold on;
plot([0,0],[-10,200],'k');
hold on;
plot([-10,200],[0,0],'k');
hold on;
axis([ -20 200 -20 200 ]) ;

% %.. Guidance Command
% figure(2) ;
% plot( t(1:end-1), u, 'LineWidth', 2 ) ;
% xlabel('Time (s)') ;
% ylabel('u (m/s^2)') ;


end