function [f,j] = CostFunWeightSelfCalibShortSwivelAHRS(cdpr_v,cdpr_p,X,k,delta_sigma,roll,pitch,delta_yaw,sensor_disturb)
% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using swivel angle
% variation measures and euler angles from an AHRS. Read Zoffoli2025
% CableCon for the complete formulation.

% set constants for the weighing matrix
sigma_max = sensor_disturb.swivel_noise;
epsilon_max = sensor_disturb.AHRS_noise;

% extract variables and parameters 
zeta_0 = X(1:cdpr_p.pose_dim);
Z = X(cdpr_p.pose_dim+1:(k+1)*cdpr_p.pose_dim);
cdpr_v = UpdateIKZeroOrd(zeta_0(1:3),zeta_0(4:6),cdpr_p,cdpr_v);
sigma_0 = zeros(cdpr_p.n_cables,1);
for j_=1:cdpr_p.n_cables
    sigma_0(j_) = cdpr_v.cable(j_).swivel_ang;
end
psi_0 = zeta_0(end);

% fill the residual vector
n = cdpr_p.n_cables;
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
J_sigma = zeros(n,cdpr_p.pose_dim,k);
for i=1:k
    zeta_k = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim);
    cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
    sigma_model=zeros(n,1);
    for j_=1:n
        sigma_model(j_) = cdpr_v.cable(j_).swivel_ang;
    end
    f_sigma(i*n-(n-1):i*n)=sigma_model-sigma_0-delta_sigma(:,i);
    f_epsilon(i*3-2:i*3) = zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0];
    J_sigma(:,:,i) = cdpr_v.analitic_jacobian_s';
end
F = [f_sigma; f_epsilon];
W = diag([ones(size(f_sigma))./(sigma_max^2); ...
            ones(size(f_epsilon))./(epsilon_max^2)]);
f = 0.5*F'*W*F;

% identification matrix (jacobian) computation
if nargout>1
    cdpr_v=UpdateIKZeroOrd(zeta_0(1:3),zeta_0(4:6),cdpr_p,cdpr_v);
    J_sigma_0 = repmat(-cdpr_v.analitic_jacobian_s',[k 1]);
    J_psi_0 = repmat(-[zeros(3), diag([0 0 1])],[k 1]);
    J_zeta = zeros((n+3)*k,cdpr_p.pose_dim*k);
    J_epsilon = [zeros(3) eye(3)];
    for i = 1:k
        J_zeta(i*n-(n-1):i*n,i*6-5:i*6) = J_sigma(:,:,i);
        J_zeta(k*n+3*i-2:k*n+3*i,i*6-5:i*6) = J_epsilon;
    end
    J_zeta_0 = [J_sigma_0;J_psi_0]; 

    J = [J_zeta_0 J_zeta];
    j = F'*W*J;
end
end