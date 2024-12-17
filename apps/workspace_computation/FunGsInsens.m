function val = FunGsInsens(cdpr_p,act_vars,un_act_vars,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = [act_vars;un_act_vars];
cdpr_v = UpdateIKZeroOrd(act_vars,un_act_vars,cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
  (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
%  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,pose(1:5),pose(6));
%  cdpr_v.underactuated_platform =...
%   cdpr_v.underactuated_platform.UpdateStatics(cdpr_p.underactuated_platform,...
%   cdpr_v.analitic_jacobian,cdpr_v.D_mat,cdpr_v.platform.ext_load);
% [cdpr_v.tension_vector,vector] = cdpr_v.underactuated_platform.CalcStaticTension(cdpr_p.underactuated_platform);

K_tot = CalcStiffnessMat(cdpr_v);
K_ort = CalcStiffnessMatOrt(cdpr_v.underactuated_platform.geometric_orthogonal,K_tot);
d = eig(K_ort);
flags.stable = all(d > 0);

dtau_dl = 100.*(-cdpr_v.underactuated_platform.geometric_parallel'*K_tot*...
  (-cdpr_v.underactuated_platform.geometric_orthogonal*...
  linsolve(K_ort,cdpr_v.underactuated_platform.geometric_orthogonal')*K_tot...
  +eye(6))*cdpr_v.underactuated_platform.geometric_parallel)./cdpr_v.tension_vector;
 %val=-norm(dtau_dl,Inf);
%val=norm(dtau_dl,Inf);
val = norm(un_act_vars);
%val = 0;

end