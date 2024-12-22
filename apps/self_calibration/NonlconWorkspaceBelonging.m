function [c,ceq] = NonlconWorkspaceBelonging(cdpr_v,cdpr_p,Z,k,ws_info)
% This is a nonlinear constraint function on a vector of poses Z to ensure
% wrench feasibility and avoid cable-to-cable and cable-to-platform 
% interferences at each pose

feasibility_flags = zeros(k,1);
for i=1:k
    pose = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim,:);
    cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
    feasibility_flags(i) = CheckWorkspaceBelonging(cdpr_p,cdpr_v,ws_info);
end

c = [];
ceq = feasibility_flags-ones(k,1);
end