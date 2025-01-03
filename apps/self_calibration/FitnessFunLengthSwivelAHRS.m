function F = FitnessFunLengthSwivelAHRS(cdpr_v,cdpr_p,Z,k,method)
%FITNESSFUNSWIVELAHRSMOTOR is a function that can be used by an optimization
% algorithm. It computes the minimum singular value of the Identification
% matrix for an initial-pose self-calibration using an AHRS for euler angles
% measurement, swivel angles variation measurements and motor angle
% variation.

Z=reshape(Z,[cdpr_p.pose_dim*k 1]);  % ga works with row vectors, flip it

% compute the jacobian matrix
X = [Z;zeros(2*cdpr_p.n_cables+1,1)];
delta_length = zeros(cdpr_p.n_cables,k);
delta_sigma = zeros(cdpr_p.n_cables,k);
delta_yaw = zeros(k,1);
roll = zeros(k,1);
pitch = zeros(k,1);
[~,J] = CostFunSelfCalibrationLengthSwivelAHRS(cdpr_v,cdpr_p,X,k,delta_length,delta_sigma,roll,pitch,delta_yaw);

% compute observabilty index
switch(method)
    
    case OptimalConfigurationMethod.MIN_SINGULAR_VALUE
        F = -min(svd(J));
    
    case OptimalConfigurationMethod.MIN_CONDITION_NUM
        F = -min(svd(J))/max(svd(J));
    
end

end