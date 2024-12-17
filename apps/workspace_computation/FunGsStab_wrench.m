function val = FunGsStab_wrench(cdpr_p,act_vars,un_act_vars,varargin,rec)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [act_vars;un_act_vars(1:3)];
cdpr_p.platform.ext_force_glob = un_act_vars(4:6);
cdpr_p.platform.ext_torque_glob = un_act_vars(7:9);
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