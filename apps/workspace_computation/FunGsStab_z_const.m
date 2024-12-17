function val = FunGsStab_z_const(cdpr_p,act_vars,un_act_vars,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [un_act_vars(1:2); act_vars;un_act_vars(3:5)];
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
K = CalcGeometricStiffnessMat(cdpr_v);

nullJ = null(cdpr_v.geometric_jacobian');
e = -eig(nullJ'*K*nullJ);

% if min(e)<0
%     val = 100000;
% else
    %val = 1/min(e);
    val = min(e);
% end

end