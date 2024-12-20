function [fun,grad] = CheckTangencyFun(cdpr_v,cdpr_p,pose,tangency_angles)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
for i=1:cdpr_p.n_cables
    tangency_angles_computation(i) = cdpr_v.cable(i).tan_ang;
end

fun = tangency_angles_computation-tangency_angles;

grad = cdpr_v.analitic_jacobian_p';
end