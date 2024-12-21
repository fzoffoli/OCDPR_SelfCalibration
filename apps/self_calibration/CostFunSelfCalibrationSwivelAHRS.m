function [F,J] = CostFunSelfCalibrationSwivelAHRS(cdpr_v,cdpr_p,X,k,delta_sigma,roll,pitch,delta_yaw)
% This is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using swivel angle
% variation measures and euler angles from an AHRS.

% extract variables and parameters 
Z = X(1:k*cdpr_p.pose_dim);
sigma_0 = X(k*cdpr_p.pose_dim+1:end-1);
psi_0 = X(end);

% fill the residual vector
n = cdpr_p.n_cables;
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
for i=1:k
    zeta_k = X(i*n-(n-1):i*n);
    cdpr_v = UpdateIKZeroOrd(zeta_k(1:3),zeta_k(4:6),cdpr_p,cdpr_v);
    for j=1:n
        sigma_model(j) = cdpr_v.cable(j).swivel_angle;
    end
    f_sigma(i*n-(n-1):i*n)=sigma_model+sigma_0+;

end

end
end