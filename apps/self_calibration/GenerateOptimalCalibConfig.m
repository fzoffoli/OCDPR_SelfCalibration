function Z = GenerateOptimalCalibConfig(cdpr_p,cdpr_v,ws_info,N,pose_bounds,method,cable_lengths_flag)
% This function compute the set of poses necessary for the initial pose
% self-calibration of an OCDPR.
% INPUT parameters, variables, pose bounds for random gen, observation
% index, flag on cable lengths usage or not.
% OUTPUT a matrix Z containing the calibration poses

% generate random poses and check for their feasibility
random_poses = zeros(cdpr_p.pose_dim,N*10);
workspace_belonging = zeros(N*10,1);
for i=1:N*10
    random_poses(:,i) = rand([cdpr_p.pose_dim 1]).*(pose_bounds(:,2)-pose_bounds(:,1)) + pose_bounds(:,1);
    cdpr_v = UpdateIKZeroOrd(random_poses(1:3,i),random_poses(4:6,i),cdpr_p,cdpr_v);
    workspace_belonging(i) = CheckWorkspaceBelonging(cdpr_p,cdpr_v,ws_info);
end

% extract the feasible poses
workspace_poses = random_poses(:,logical(workspace_belonging));
assert(length(workspace_poses)>N);

% determine the subset of poses that optimize the obesrvability index


Z=0;
end