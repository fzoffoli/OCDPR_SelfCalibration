function [F,J] = CostFunSelfCalibrationLengthSwivelAHRS(cdpr_v,cdpr_p,X,k,delta_length,delta_sigma,roll,pitch,delta_yaw)
% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using cable length 
% variation, swivel angle variation measures and euler angles from an AHRS.
% Read Zoffoli2025 CableCon for the complete formulation.

% extract variables and parameters 
Z = X(1:k*cdpr_p.pose_dim);
length_0 = X(k*cdpr_p.pose_dim+1:k*cdpr_p.pose_dim+cdpr_p.n_cables);
sigma_0 = X(k*cdpr_p.pose_dim+cdpr_p.n_cables+1:end-1);
psi_0 = X(end);

% fill the residual vector
n = cdpr_p.n_cables;
f_length = zeros(k*n,1);
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
J_length = zeros(n,cdpr_p.pose_dim,k);
J_sigma = zeros(n,cdpr_p.pose_dim,k);
for i=1:k
    zeta_k = Z(i*cdpr_p.pose_dim-(cdpr_p.pose_dim-1):i*cdpr_p.pose_dim);
    cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
    length_model = zeros(n,1);
    sigma_model=zeros(n,1);
    for j=1:n
        length_model(j) = cdpr_v.cable(j).complete_length;
        sigma_model(j) = cdpr_v.cable(j).swivel_ang;
    end
    % length_model = cdpr_v.cable_vector;
    f_length(i*n-(n-1):i*n) = length_model-length_0-delta_length(:,i); 
    f_sigma(i*n-(n-1):i*n) = sigma_model-sigma_0-delta_sigma(:,i);
    f_epsilon(i*3-2:i*3) = zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0];
    J_length(:,:,i) = cdpr_v.analitic_jacobian_l';
    J_sigma(:,:,i) = cdpr_v.analitic_jacobian_s';
end
F = [f_length; f_sigma; f_epsilon];

% identification matrix (jacobian) computation
if nargout>1
    J_psi_0 = zeros(3*k,1);
    J_length_0 = zeros(n*k,n);
    J_sigma_0 = zeros(n*k,n);
    J_zeta = zeros((2*n+3)*k,cdpr_p.pose_dim*k);
    J_epsilon = [zeros(3) eye(3)];
    for i = 1:k
        J_psi_0(i*3-2:i*3) = [0;0;-1]; 
        J_length_0(i*n-(n-1):i*n,:) = -eye(n);
        J_sigma_0(i*n-(n-1):i*n,:) = -eye(n);
        J_zeta(i*n-(n-1):i*n,i*6-5:i*6) = J_length(:,:,i);
        J_zeta(k*n+i*n-(n-1):k*n+i*n,i*6-5:i*6) = J_sigma(:,:,i);
        J_zeta(k*2*n+3*i-2:k*2*n+3*i,i*6-5:i*6) = J_epsilon;
    end
    J_psi_0 = [zeros(n*2*k,1);J_psi_0];
    J_sigma_0 = [zeros(n*k,n); J_sigma_0; zeros(3*k,n)];
    J_length_0 = [J_length_0; zeros((3+n)*k,n)];

    J = [J_zeta J_length_0 J_sigma_0 J_psi_0];
end
end