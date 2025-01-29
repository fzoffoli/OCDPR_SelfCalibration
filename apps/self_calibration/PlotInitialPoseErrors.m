function PlotInitialPoseErrors(out)
%PLOTINITIALPOSEERRORS plots the bar graph relative to the position error
%norm and orientation error norm at the end of the calibration for an OCDPR
%with different set of sensors

% extract the data from the structure
for i = 1:size(out,1)
    for j = 1:size(out,2)
        % extract swivel and AHRS simulation data
        eps_p(i,j) = out(i,j).InitialPositionErrorNorm;
        eps_p_max(i,j) = out(i,j).InitPosErrNorm_max;
        eps_p_min(i,j) = out(i,j).InitPosErrNorm_min;
        eps_r(i,j) = out(i,j).InitialOrientationError;
        eps_r_max(i,j) = out(i,j).InitOrientErr_max;
        eps_r_min(i,j) = out(i,j).InitOrientErr_min;
        k(j)     = out(i,j).NumberOfMeasures;
    end
end

figure()
b_max = bar(k,eps_p_max);
b_max(1).FaceColor = [128/255 0/255 128/255];
b_max(2).FaceColor = [255/255 165/255 0/255];
b_max(3).FaceColor = [0/255 128/255 128/255];
b_max(1).FaceAlpha = 0.5;
b_max(2).FaceAlpha = 0.5;
b_max(3).FaceAlpha = 0.5;
hold on
b = bar(k,eps_p);
b(1).FaceColor = [128/255 0/255 128/255];
b(2).FaceColor = [255/255 165/255 0/255];
b(3).FaceColor = [0/255 128/255 128/255];
b(1).FaceAlpha = 0.75;
b(2).FaceAlpha = 0.75;
b(3).FaceAlpha = 0.75;
b_min = bar(k,eps_p_min);
b_min(1).FaceColor = [128/255 0/255 128/255];
b_min(2).FaceColor = [255/255 165/255 0/255];
b_min(3).FaceColor = [0/255 128/255 128/255];
xlabel('number of poses');
ylabel('[m]')
legend('$\sigma$, $\varepsilon$','$l$, $\sigma$, $\varepsilon$','$\tau$, $\sigma$, $\varepsilon$','Interpreter','Latex')
title('Position error norm')

figure()
b_max = bar(k,eps_r_max);
b_max(1).FaceColor = [128/255 0/255 128/255];
b_max(2).FaceColor = [255/255 165/255 0/255];
b_max(3).FaceColor = [0/255 128/255 128/255];
b_max(1).FaceAlpha = 0.5;
b_max(2).FaceAlpha = 0.5;
b_max(3).FaceAlpha = 0.5;
hold on
b = bar(k,eps_r);
b(1).FaceColor = [128/255 0/255 128/255];
b(2).FaceColor = [255/255 165/255 0/255];
b(3).FaceColor = [0/255 128/255 128/255];
b(1).FaceAlpha = 0.75;
b(2).FaceAlpha = 0.75;
b(3).FaceAlpha = 0.75;
b_min = bar(k,eps_r_min);
b_min(1).FaceColor = [128/255 0/255 128/255];
b_min(2).FaceColor = [255/255 165/255 0/255];
b_min(3).FaceColor = [0/255 128/255 128/255];
xlabel('number of poses');
ylabel('[deg]')
legend('$\sigma$, $\varepsilon$','$l$, $\sigma$, $\varepsilon$','$\tau$, $\sigma$, $\varepsilon$','Interpreter','Latex')
title('Orientation error norm')

end