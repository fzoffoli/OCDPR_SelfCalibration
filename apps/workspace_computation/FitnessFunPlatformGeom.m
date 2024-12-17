% Fitness function for a Genetic Algorithm for design optimization
% INPUT starting inertial and geometric params, index of cables force
% controlled
% OUTPUT volume of the HPFT ws
function F=FitnessFunPlatformGeom(X,cc_idx,cdpr_p,cdpr_v,out,util,folder,rec,ws_info)

X=reshape(X,[8 3]);
for i = 1:cdpr_p.n_cables
    cdpr_p.cable(i).pos_PA_loc=X(i,:)';
end

out = CalcVolHPFT(cc_idx,cdpr_p,cdpr_v,out,util,folder,rec,ws_info);
F=out.V;
end