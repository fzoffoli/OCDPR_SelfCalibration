function [c,ceq] = WS_constr_calz(cdpr_p,act_vars,un_act_vars,tau_lim,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);

cdpr_v = UpdateIKZeroOrd(act_vars,un_act_vars,cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
nullJ = null(cdpr_v.geometric_jacobian');

K = CalcGeometricStiffnessMat(cdpr_v);

e = -eig(nullJ'*K*nullJ);

    
ceq = norm(nullJ'*cdpr_v.platform.ext_load);

c = [-(cdpr_v.tension_vector-tau_lim(1));
    cdpr_v.tension_vector-tau_lim(2)];


end