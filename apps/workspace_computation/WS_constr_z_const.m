function [c,ceq] = WS_constr_z_const(cdpr_p,act_vars,un_act_vars)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [un_act_vars(1:2); act_vars;un_act_vars(3:5)];
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
nullJ = null(cdpr_v.geometric_jacobian');

K = CalcGeometricStiffnessMat(cdpr_v);

e = -eig(nullJ'*K*nullJ);

    
ceq = nullJ'*cdpr_v.platform.ext_load;

c = -cdpr_v.tension_vector;
c = [c;-e];

end