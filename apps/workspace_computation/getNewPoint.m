%algorithm to find the next iteration point in the boundary flooding
%algorithm, NB in the actual implementation the already found boundary
%points are not considered in the future point direction: TODO fix the code
%accordingly to the paper
%INPUT last computed point, point of attraction, matrix of boundary points,
%all grid points
%OUTPUT next iteration point

function [Q_end, Q_comp_idx] = getNewPoint(Q_init,Q_ak,B,iter,grid_discrete)

% mapping into unitary space
ext_limits = [min(grid_discrete(1,:)) max(grid_discrete(1,:));
    min(grid_discrete(2,:)) max(grid_discrete(2,:));
    min(grid_discrete(3,:)) max(grid_discrete(3,:))];
% H_ak = CartesianToUnitary(Q_ak,ext_limits);
% H_init = CartesianToUnitary(Q_init,ext_limits);
% for i = 1:size(B,2)
%     H_b(:,i) = CartesianToUnitary(B(:,i),ext_limits);
% end

% direction given by past boundary points
d_b = zeros(3,1);
if ~isempty(B)
    for j = 1:size(B,2)
        c_bj = norm(ones(3,1)-(H_init-H_b(:,j)));
        d_bj = (H_init-H_b(:,j))/norm(H_init-H_b(:,j));
        d_b = d_b+c_bj*d_bj;
    end
    d_b = d_b/norm(d_b);
end

% direction given by the attractive point
% d_a = (H_init-H_ak)/norm(H_init-H_ak);

% exploration direction in the unitary space
tau = 0.05;      % parameter to be checked
% c_a = 1-exp(-iter/tau);
% c_b = exp(-iter/tau);
% d_u = (c_a*d_a - c_b*d_b)/norm(c_a*d_a - c_b*d_b);
d_u = (Q_ak-Q_init)/norm(Q_ak-Q_init);

% exploration direction in the task space
% d_T = UnitaryToCartesian(d_u,ext_limits);
delta_x = diff(unique(grid_discrete(1,:)));
delta_y = diff(unique(grid_discrete(2,:)));
delta_z = diff(unique(grid_discrete(3,:)));
d_T = d_u.*[delta_x(1);delta_y(1);delta_z(1)];

% closer point in the grid
Q_end_ideal = Q_init+d_T;
for k = 1:length(grid_discrete)
    distance(k) = norm(Q_end_ideal-grid_discrete(:,k));
end
[~,Q_comp_idx] = min(distance);
Q_end = grid_discrete(:,Q_comp_idx); % Fix code, the procedure doesn't work from the first iteration

end