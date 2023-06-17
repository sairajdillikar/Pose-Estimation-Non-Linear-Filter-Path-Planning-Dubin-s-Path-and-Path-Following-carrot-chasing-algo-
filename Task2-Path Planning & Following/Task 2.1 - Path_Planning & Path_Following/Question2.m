% Carrot Chasing algorithm

clear all;
close all;
clc;

% Initialisation of the waypoints
waypoints = [[0, 10,0]; 
             [60, 60,45]; 
             [80, 120,30]; 
             [150, 70, -90]; 
             [100, 30,-120]; 
             [50,0,-180];]';

% Initialising the variables;
radius = 5;     % Turning Radius
Cur_pos = [waypoints(1,1);waypoints(2,1)]; % Initial Position of the Robot
Cur_head_ang = [waypoints(3,1)]; % Initial Heading of the Robot

% Dubins_Path function to obtain the exit & entry (angles, circles & points) along with the path combition 
[exit_c,entry_c,exit_angle,entry_angle,exit_p,entry_p,Path_Combi] = Dubins_Path(waypoints,radius);

% Loop to construct a path using carrot-chasing algorithm for the generated dubins path
for i=1:1:5

    if Path_Combi(i) == 1
    fg = 1;
    fl = 1;
    elseif Path_Combi(i) == 2
    fg = 1;
    fl = -1;
    elseif Path_Combi(i) == 3
    fg = -1;
    fl = 1;
    elseif Path_Combi(i) == 4
    fg = -1;
    fl = -1;
    end

    % Carrot-Chasing algorithm function calling
    [Cur_pos, Cur_head_ang] = CCA_Circular_Motion(exit_c(:,i),exit_p(:,i),Cur_pos(:,1),Cur_head_ang(1),45,fg,0.15);
    [Cur_pos, Cur_head_ang] = CCA_Circular_Motion(entry_c(:,i),entry_p(:,i),Cur_pos(:,1),Cur_head_ang(1),45,fl,0.15);
    
end