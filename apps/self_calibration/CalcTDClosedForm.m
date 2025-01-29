function tau = CalcTDClosedForm(cdpr_v,cdpr_p,tension_bounds)
%This tension distribution algorithm was designed based on "Pott2018"

cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
tau_m = ones(cdpr_p.n_cables,1)*(tension_bounds(1)+tension_bounds(2))/2;
w = cdpr_v.platform.ext_load;
b = w-cdpr_v.geometric_jacobian_l*tau_m;

tau_v = lsqminnorm(cdpr_v.geometric_jacobian_l,b);

tau = tau_v+tau_m;
end