close all
clear all
%clearvars
clc

x_max = 1000;
y_max = 1000;

poly = { [300,500; 200,200;450,300],[310,750;480,650;500,550;340,600],[600,700;650,500;750,600;800,780],[550,450;750,350;700,250;530,300]};
      

EPS = 30;
numNodes = 1000; 

%% Initialization %%%%%%%%%%%
q_start.coord = [0,0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [600,600];
q_goal.cost = 0;

nodes(1) = q_start;
%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
axis([-5 x_max -5 y_max])
hold on
plot(q_start.coord(1),q_start.coord(2),'r*','LineWidth',2)
plot(q_goal.coord(1),q_goal.coord(2),'g*','LineWidth',2)
RRT_obstacles_plot_poly(gca, poly);

%%
for i = 1:1:numNodes


    %% Step 1:Generate Random state or points 
    %%%%==========================================
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];

    %%%%% Break if goal node is already reached
    for j = 1:1:length(nodes)
        if dist(nodes(j).coord, q_goal.coord) <= 50   %%%%nodes(j).coord == q_goal.coord
            break
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %% Step 2: Pick the closest node from existing list to branch out from
    %%%%%==================================================================
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord,q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_nearest = nodes(idx);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %% Step 3: Steer from 𝑞_𝑛𝑒𝑎𝑟𝑒𝑠𝑡 to 𝑞_𝑟𝑎𝑛𝑑 
    %%%%==========================================
    q_new.coord = steer(q_rand, q_nearest.coord,val,EPS);
    edge_rand = [q_nearest.coord(1),q_nearest.coord(2); q_new.coord];
    plot(q_rand(1),q_rand(2),'r.')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %% Step 4: Collision check %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%===================================================
    ln= [q_nearest.coord(1),q_nearest.coord(2); q_new.coord(1),q_new.coord(2)];
    chk_collision(ln,poly);
    
    if chk_collision(ln,poly) == 0  
        line([q_nearest.coord(1), q_new.coord(1)], [q_nearest.coord(2), q_new.coord(2)], 'Color', 'y', 'LineWidth', 1);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_nearest.coord) + q_nearest.cost;
        q_new.parent = idx;

  
        %% Step 5: Append to nodes %%%%%%%%%%%
        %%%%====================================
        nodes = [nodes q_new];
    end
end


%% Find Path : Search backwards from goal to start to find the optimal least cost path
%%%%=================================================================================

%%%% Step 1:
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord,q_goal.coord);
    D = [D tmpdist];
end

%%%%Step 2:
[val, idx] = min(D);


q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
k =0;
while q_end.parent ~= 0
    start = q_end.parent;
    k=k+1;
    via_points(k,:) = nodes(start);
    
    %%%%Step 3:
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'g', 'LineWidth', 3);
    via_points = flipud(via_points);    

    %%%% Step 4:
    q_end = nodes(start);
end

title(sprintf('EPS: %g, Number of Nodes: %g, Cost: %g', EPS, numNodes, n.cost));



