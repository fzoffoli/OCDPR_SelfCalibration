function PlotInitialPoseErrors(out)
%PLOTINITIALPOSEERRORS plots the bar graph relative to the position error
%norm and orientation error norm at the end of the calibration for an OCDPR
%with different set of sensors

% extract the data from the structure
for j = 1:size(out,2)
    % extract swivel and AHRS simulation data 
    swivel_ahrs_eps_p(j) = out(1,j).InitialPositionErrorNorm;
    swivel_ahrs_eps_r(j) = out(1,j).InitialOrientationError;
    swivel_ahrs_k(j)     = out(1,j).NumberOfMeasures;

    % extract length, swivel and AHRS simulation data
    length_swivel_ahrs_eps_p(j) = out(2,j).InitialPositionErrorNorm;
    length_swivel_ahrs_eps_r(j) = out(2,j).InitialOrientationError;
    length_swivel_ahrs_k(j)     = out(2,j).NumberOfMeasures;

    % extract loadcell, swivel and AHRS simulation data
    loadcell_swivel_ahrs_eps_p(j) = out(3,j).InitialPositionErrorNorm;
    loadcell_swivel_ahrs_eps_r(j) = out(3,j).InitialOrientationError;
    loadcell_swivel_ahrs_k(j)     = out(3,j).NumberOfMeasures;
end

figure()
bar(swivel_ahrs_k,swivel_ahrs_eps_p)
hold on
bar(length_swivel_ahrs_k,length_swivel_ahrs_eps_p)
hold on
bar(loadcell_swivel_ahrs_k,loadcell_swivel_ahrs_eps_p)
legend('Swivel AHRS','Lenght Swivel AHRS','Loadcell SWivel AHRS')
title('Position error norm')

figure()
bar(swivel_ahrs_k,swivel_ahrs_eps_r)
hold on
bar(length_swivel_ahrs_k,length_swivel_ahrs_eps_r)
hold on
bar(loadcell_swivel_ahrs_k,loadcell_swivel_ahrs_eps_r)
legend('Swivel AHRS','Lenght Swivel AHRS','Loadcell SWivel AHRS')
title('Orientation error norm')

end