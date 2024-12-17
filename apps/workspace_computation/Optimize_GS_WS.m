function [pose,out] = Optimize_GS_WS(cdpr_p,cdpr_v,ut,pose,tau_lim,ws_info,out,varargin)

ws_lim = ws_info.limits;

[x,fval,exitflag,output] = fmincon(@(x)FunGsInsens(cdpr_p,pose(1:3),x),pose(4:6),[],[],[],[],[],[],@(x)WS_constr(cdpr_p,pose(1:3),x,tau_lim,varargin{1}),ut.fmincon_options);


if output.constrviolation<0.1
   
    pose = [pose(1:3);x];
    out.counter = out.counter+1;
    out.pose(:,out.counter) = pose;
    out.position(:,out.counter) = out.pose(1:3,out.counter);
    out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
    cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

    varargin{1}.SetFrame(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
    nullJ = null(cdpr_v.geometric_jacobian');
    out.constr(1,out.counter) = norm(nullJ'*cdpr_v.platform.ext_load);
    out.tension_vector(:,out.counter)= cdpr_v.tension_vector;
    out.cable_length(:,out.counter) = cdpr_v.cable_vector;
    K = CalcGeometricStiffnessMat(cdpr_v);

e = -eig(nullJ'*K*nullJ);

%     out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
%     out.nat_freq(:,out.counter) = sqrt(diag(eigenvalues_mat))./(2.*pi);
%     out.normal_modes(:,:,out.counter) = eigenvectors;
%     out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
out.WS_perf(1,out.counter) = fval;
if (fval < 0.5)
   e = 0; 
 end
end

end