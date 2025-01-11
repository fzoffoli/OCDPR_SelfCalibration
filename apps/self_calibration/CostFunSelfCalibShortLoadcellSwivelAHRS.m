function [F,J] = CostFunSelfCalibShortLoadcellSwivelAHRS(cdpr_v,cdpr_p,X,k,tau,delta_sigma,roll,pitch,delta_yaw)
% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using loadcells 
% measures, swivel angle variation measures and euler angles from an AHRS.
% Read Zoffoli2025 CableCon for the complete formulation.

% extract variables and parameters 
zeta_0 = X(1:cdpr_p.pose_dim);
Z = X(cdpr_p.pose_dim+1:cdpr_p.pose_dim*(k+1));
cdpr_v = UpdateIKZeroOrd(zeta_0(1:3),zeta_0(4:6),cdpr_p,cdpr_v);
sigma_0 = zeros(cdpr_p.n_cables,1);
for j=1:cdpr_p.n_cables
    sigma_0(j) = cdpr_v.cable(j).swivel_ang;
end
psi_0 = zeta_0(end);

% fill the residual vector
n = cdpr_p.n_cables;
f_tau = zeros(k*6,1);
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
for i=1:k
    zeta_k = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim);
    cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    length_model = zeros(n,1);
    sigma_model=zeros(n,1);
    for j=1:n
        length_model(j) = cdpr_v.cable(j).complete_length;
        sigma_model(j) = cdpr_v.cable(j).swivel_ang;
    end
    % length_model = cdpr_v.cable_vector;
    f_tau(i*6-(6-1):i*6) = cdpr_v.geometric_jacobian_l*tau(:,i)-cdpr_v.platform.ext_load; 
    f_sigma(i*n-(n-1):i*n) = sigma_model-sigma_0-delta_sigma(:,i);
    f_epsilon(i*3-2:i*3) = zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0];
end
F = [f_tau; f_sigma; f_epsilon];
end