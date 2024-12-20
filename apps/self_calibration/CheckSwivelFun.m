function [fun,grad] = CheckSwivelFun(cdpr_v,cdpr_p,pose,swivel_angles)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
for i=1:cdpr_p.n_cables
    swivel_angles_computation(i) = cdpr_v.cable(i).swivel_ang;
end

fun = swivel_angles_computation-swivel_angles;

grad = cdpr_v.analitic_jacobian_s';
end