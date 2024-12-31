function [F,J] = bar(cdpr_v,cdpr_p,pose)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
% F(1) = cdpr_v.cable(1).theta_motor;
F(1) = cdpr_v.cable(1).complete_length;
if nargout>1
    % J = cdpr_v.cable(1).analitic_jacobian_l_col/cdpr_v.cable(1).l_theta_deriv;
    % J = cdpr_v.cable(1).analitic_jacobian_l_col/cdpr_v.cable(1).l_theta_deriv;
    J = zeros(6,1);
end
end