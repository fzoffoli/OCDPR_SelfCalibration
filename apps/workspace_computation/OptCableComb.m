%INPUT geometric and inertial parameters of the ocdpr
%OUTPUT index of cable couple that maximize the HPFT ws
function [cable_comb_idx, cdpr_outputs] = OptCableComb(cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info)

cableComb=nchoosek(1:cdpr_parameters.n_cables,cdpr_parameters.n_cables-cdpr_parameters.pose_dim);
V=zeros(length(cableComb),1);
parfor i=1:length(cableComb)
    out(i) = CalcVolHPFT(i,cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info);
    V(i)=out(i).V;
end

[~,cable_comb_idx]=max(V);
cdpr_outputs=out(cable_comb_idx);
end