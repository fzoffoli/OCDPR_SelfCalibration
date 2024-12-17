function [toDo, B] = SpaceExploration(toDo,Q_ak,B,Q_start,Q_start_idx,grid_discrete,cdpr_p,cdpr_v,ut,ws_info,out,CablesForceControlled,rec)

Q_init = Q_start;
Q_init_idx = Q_start_idx;
iter = 0;
flag = true;
orient = [0;0;0];
old_counter = 0;
Q_end = zeros(3,1);

% uncomment for graphics
% figure()
% scatter3(Q_init(1),Q_init(2),Q_init(3),'filled');
% hold on
% scatter3(Q_ak(1),Q_ak(2),Q_ak(3),'filled');
while (flag &&  ~isequal(Q_end,Q_ak))   % TODO change the feasibility conditions including the reachable ws (also in the ws bottom)
    [Q_end, Q_comp_idx] = getNewPoint(Q_init,Q_ak,B,iter,grid_discrete);
    out = getWSconditions(cdpr_p,cdpr_v,Q_end,orient,ws_info,ut,out,CablesForceControlled,rec);
    flag = out.flag;
    old_counter = out.counter(end);
    if (flag &&  ~isequal(Q_end,Q_ak)) 
        %save results
        iter = iter+1;
        Q_init = Q_end;
        Q_init_idx = Q_comp_idx;

        % uncomment for graphics
        % hold on
        % scatter3(Q_end(1),Q_end(2),Q_end(3),'filled');
    end
end
toDo(end+1).b_points = [Q_init, Q_end];
toDo(end).b_points_idx = [Q_init_idx, Q_comp_idx];
B(:,end+1) = Q_end;

end