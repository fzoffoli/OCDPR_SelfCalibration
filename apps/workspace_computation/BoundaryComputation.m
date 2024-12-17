% this function computes the boundary of a close workspace starting from
% two adjacent points inside toDo, one belonging to the ws and the other not
%INPUT toDo points
%OUTPUT matrix containing the boundary feasible points

function feasible_points = BoundaryComputation(toDo,feasible_points,unfeasible_points,neighbour,grid_discrete,cdpr_p,cdpr_v,ut,ws_info,out,CablesForceControlled,rec)

orient = [0;0;0];
Q_g_idx = toDo(end).b_points_idx(1);
Q_i_idx = toDo(end).b_points_idx(2);
feasible_points(Q_g_idx) = 1;
unfeasible_points(Q_i_idx) = 1;

% iterations continue untill there aren't unfeasible points near feasible
% ones, thus the toDo set is empty
while ~isempty(toDo)
    % Q_g = toDo(end).b_points(:,1);
    Q_i = toDo(end).b_points(:,2);
    N_cnt=0;
    % compute the neighbourhood points not yet computed
    for i = 1:length(grid_discrete)
        distance = norm(Q_i-grid_discrete(:,i));
        if distance<neighbour && ~feasible_points(i) && ~unfeasible_points(i)
            N_cnt = N_cnt + 1;
            N(:,N_cnt) = grid_discrete(:,i);
            N_idx(N_cnt) = i;
        end
    end
    % erase last toDo points from the analysis
    toDo(end) = [];
    old_counter = out.counter(end);
    while ~isempty(N)   %TODO debug the code from here!
        % check if there are feasible points close to the neighboourhood ones
        N_w = [];
        for j = 1:length(grid_discrete)
            distance = norm(N(:,end)-grid_discrete(:,j));
            if distance<neighbour && feasible_points(j)
                N_w = grid_discrete(:,j);
            end
        end
        if ~isempty(N_w)
            out = getWSconditions(cdpr_p,cdpr_v,N(:,end),orient,ws_info,ut,out,CablesForceControlled,rec);
            flag = out.flag;
            old_counter = out.counter(end);
            if flag
                % update feasible points
                feasible_points(N_idx(end))=1;
                if (N(1,end) == min(grid_discrete(1,:)) || N(1,end) == max(grid_discrete(1,:)) || ...
                    N(2,end) == min(grid_discrete(2,:)) || N(1,end) == max(grid_discrete(2,:)) || ...
                    N(3,end) == min(grid_discrete(3,:)) || N(1,end) == max(grid_discrete(3,:)))
                    toDo(end+1).b_points(:,2) = N(:,end);
                end
            else
                % update unfeasible points and toDo list
                unfeasible_points(N_idx(end))=1;
                toDo(end+1).b_points(:,2)=N(:,end);
            end
            N_w = [];
        end
        N(:,end) = [];
        N_idx(end) = [];
    end
end
% uncomment for graphical representation
% figure()
% scatter3(grid_discrete(1,find(feasible_points)),grid_discrete(2,find(feasible_points)),grid_discrete(3,find(feasible_points)),'filled')
% hold on
% scatter3(grid_discrete(1,find(unfeasible_points)),grid_discrete(2,find(unfeasible_points)),grid_discrete(3,find(unfeasible_points)))
end