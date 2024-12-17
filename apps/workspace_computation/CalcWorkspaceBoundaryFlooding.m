% This function calculates the boundary points of the workspace defined by a boolean function 
% with the Flooding Algorithm proposed by Zaccaria in 2023

function out = CalcWorkspaceBoundaryFlooding(cdpr_p,ws_info,cdpr_v,ut,out,CablesForceControlled,rec)

% initialize grid
ext_limits = [cdpr_p.cable(1).pos_OD_glob(1) cdpr_p.cable(1).pos_OD_glob(1);
    cdpr_p.cable(1).pos_OD_glob(2) cdpr_p.cable(1).pos_OD_glob(2);
    cdpr_p.cable(1).pos_OD_glob(3) cdpr_p.cable(1).pos_OD_glob(3)];
for i = 2:cdpr_p.n_cables
    ext_limits = [min([cdpr_p.cable(i).pos_OD_glob(1) ext_limits(1,1)]) max([cdpr_p.cable(i).pos_OD_glob(1) ext_limits(1,2)]);
        min([cdpr_p.cable(i).pos_OD_glob(2) ext_limits(2,1)]) max([cdpr_p.cable(i).pos_OD_glob(2) ext_limits(2,2)]); 
        min([cdpr_p.cable(i).pos_OD_glob(3) ext_limits(3,1)]) max([cdpr_p.cable(i).pos_OD_glob(3) ext_limits(3,2)])];
end
% x_discrete = linspace(ext_limits(1,1),ext_limits(1,2),ws_info.mesh_divider);
% y_discrete = linspace(ext_limits(2,1),ext_limits(2,2),ws_info.mesh_divider);
% z_discrete = linspace(ext_limits(3,1),ext_limits(3,2),ws_info.mesh_divider);
% grid_discrete = zeros(3,ws_info.mesh_divider^3);
% equidistant grid is more effective (cubic discretization)
grid_pitch = 0.4;
x_discrete = ext_limits(1,1):grid_pitch:ext_limits(1,2);
y_discrete = ext_limits(2,1):grid_pitch:ext_limits(2,2);
z_discrete = ext_limits(3,1):grid_pitch:ext_limits(3,2);
grid_discrete = zeros(3,length([x_discrete y_discrete z_discrete]));

cnt = 1;
for i = 1:length(x_discrete)
    for j = 1:length(y_discrete)
        for k = 1:length(z_discrete)
            grid_discrete(:,cnt) = [x_discrete(i);y_discrete(j);z_discrete(k)];
            cnt = cnt+1;
        end
    end
end
neighbour = norm([x_discrete(2)-x_discrete(1); y_discrete(2)-y_discrete(1); z_discrete(2)-z_discrete(1)]) + 1e-3;

% starting point definition (to be placed inside the WS)
Q_start_ideal = zeros(3,1);
distance = zeros(length(grid_discrete),1);
for r = 1:length(grid_discrete)
    distance(r) = norm(Q_start_ideal-grid_discrete(:,r));
end
[~,Q_start_idx] = min(distance);
Q_start = grid_discrete(:,Q_start_idx);

exploration_number = 1;     % 24 to copy Zaccaria
boundary_idx = boundary(grid_discrete(1,:)',grid_discrete(2,:)',grid_discrete(3,:)');
A = zeros(3,exploration_number);    % TODO check if the boundary points are outside the reachable ws
B = [];
toDo = [];
out.counter = 0;
feasible_points = zeros(length(grid_discrete),1);
unfeasible_points = zeros(length(grid_discrete),1);
for k = 1:exploration_number
    % generation of attractive points
    temp_point_idx = randi(length(boundary_idx));
    A(:,k) = [grid_discrete(1,boundary_idx(temp_point_idx,1));
        grid_discrete(2,boundary_idx(temp_point_idx,2));
        grid_discrete(3,boundary_idx(temp_point_idx,3))];


    toDo = SpaceExploration(toDo,A(:,k),B,Q_start,Q_start_idx,grid_discrete,cdpr_p,cdpr_v,ut,ws_info,out,CablesForceControlled,rec);
    feasible_points = BoundaryComputation(toDo,feasible_points,unfeasible_points,neighbour,grid_discrete,cdpr_p,cdpr_v,ut,ws_info,out,CablesForceControlled,rec);
end

[k,out.vol]=boundary(grid_discrete(1,find(feasible_points))',grid_discrete(2,find(feasible_points))',grid_discrete(3,find(feasible_points))',1);
% uncomment for graphical representation
% figure()
% trisurf(k,grid_discrete(1,find(feasible_points))',grid_discrete(2,find(feasible_points))',grid_discrete(3,find(feasible_points))')
end