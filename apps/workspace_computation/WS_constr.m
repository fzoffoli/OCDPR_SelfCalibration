function [c,ceq] = WS_constr(cdpr_p,act_vars,un_act_vars,tau_lim,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [act_vars;un_act_vars];
cdpr_v = UpdateIKZeroOrd(act_vars,un_act_vars,cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
%  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,pose(1:5),pose(6));
%  cdpr_v.underactuated_platform =...
%   cdpr_v.underactuated_platform.UpdateStatics(cdpr_p.underactuated_platform,...
%   cdpr_v.analitic_jacobian,cdpr_v.D_mat,cdpr_v.platform.ext_load);
% [cdpr_v.tension_vector,vector] = cdpr_v.underactuated_platform.CalcStaticTension(cdpr_p.underactuated_platform);

nullJ = null(cdpr_v.geometric_jacobian');

K = CalcGeometricStiffnessMat(cdpr_v);

e = -eig(nullJ'*K*nullJ);

    
ceq = nullJ'*cdpr_v.platform.ext_load;

c = [-(cdpr_v.tension_vector-tau_lim(1));
    cdpr_v.tension_vector-tau_lim(2);
    -e;];


end