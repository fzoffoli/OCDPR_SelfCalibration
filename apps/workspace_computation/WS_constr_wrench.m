function [c,ceq] = WS_constr_wrench(cdpr_p,act_vars,un_act_vars,rec)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [act_vars;un_act_vars(1:3)];
cdpr_p.platform.ext_force_glob = un_act_vars(4:6);
cdpr_p.platform.ext_torque_glob = un_act_vars(7:9);
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
nullJ = null(cdpr_v.geometric_jacobian');

K = CalcGeometricStiffnessMat(cdpr_v);

e = -eig(nullJ'*K*nullJ);

    
ceq = nullJ'*cdpr_v.platform.ext_load;

c = -cdpr_v.tension_vector;
ceq = [ceq;min(e)];

end