function [psi,J] = foobar(cdpr_v,cdpr_p,position,orientation)
    
cdpr_v = UpdateIKZeroOrd(position,orientation,cdpr_p,cdpr_v);
psi = cdpr_v.cable(1).tan_ang;

if nargout >1
    J=cdpr_v.cable(1).analitic_jacobian_p_col(1:3);
end

end