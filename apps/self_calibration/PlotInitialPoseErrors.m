function PlotInitialPoseErrors(out)
%PLOTINITIALPOSEERRORS plots the bar graph relative to the position error
%norm and orientation error norm at the end of the calibration for an OCDPR
%with different set of sensors

% extract the data from the structure
for i = 1:size(out,1)
    for j = 1:size(out,2)
        % extract swivel and AHRS simulation data
        eps_p(i,j) = out(i,j).InitialPositionErrorNorm;
        eps_r(i,j) = out(i,j).InitialOrientationError;
        k(j)     = out(i,j).NumberOfMeasures;
    end
end

figure()
bar(k,eps_p)
legend('Swivel AHRS','Lenght Swivel AHRS','Loadcell Swivel AHRS')
title('Position error norm')

figure()
bar(k,eps_r)
legend('Swivel AHRS','Lenght Swivel AHRS','Loadcell Swivel AHRS')
title('Orientation error norm')

end