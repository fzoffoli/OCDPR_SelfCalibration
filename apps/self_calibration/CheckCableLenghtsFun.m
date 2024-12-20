function [fun,grad] = CheckCableLenghtsFun(cdpr_v,cdpr_p,pose,cable_lengths)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
for i=1:cdpr_p.n_cables
    cable_lengths_computation(i) = cdpr_v.cable(i).complete_length;
end

fun = cable_lengths_computation-cable_lengths;

grad = cdpr_v.analitic_jacobian_l';
end