function JacobiansCheck(cdpr_parameters,cdpr_variables)

dt = 0.001;
Tmax = 5;
t = 0:dt:Tmax;
zeta_0 = [-0.5;0;0.5;0;0;0];
zeta_1 = [0.5;0;-0.5;0;pi/6;0];

zeta_d_numeric = zeros(cdpr_parameters.pose_dim,length(t));
sigma_d_numeric = zeros(cdpr_parameters.n_cables,length(t));
psi_d_numeric = zeros(cdpr_parameters.n_cables,length(t));
l_d_numeric = zeros(cdpr_parameters.n_cables,length(t));

sigma_d_analytic = zeros(cdpr_parameters.n_cables,length(t));
psi_d_analytic = zeros(cdpr_parameters.n_cables,length(t));
l_d_analytic = zeros(cdpr_parameters.n_cables,length(t));

for i = 1:length(t)
    zeta(:,i) = zeta_0+(zeta_1-zeta_0)*t(i)/Tmax;
    cdpr_variables = UpdateIKZeroOrd(zeta(1:3,i),zeta(4:6,i),cdpr_parameters,cdpr_variables);
    for j = 1:cdpr_parameters.n_cables
        sigma(j) = cdpr_variables.cable(j).swivel_ang;
        psi(j) = cdpr_variables.cable(j).tan_ang;
        l(j) = cdpr_variables.cable(j).complete_length;
    end
    
    %calculate numeric derivatives
    if i~=1
        zeta_d_numeric(:,i) = (zeta(:,i)-zeta_old)/dt;
        sigma_d_numeric(:,i) = (sigma-sigma_old)/dt;
        psi_d_numeric(:,i) = (psi-psi_old)/dt;
        l_d_numeric(:,i) = (l-l_old)/dt;
    end
    
    %calculate analytic derivatives
    sigma_d_analytic(:,i) = cdpr_variables.analitic_jacobian_s'*(zeta_1-zeta_0)/Tmax;
    psi_d_analytic(:,i) = cdpr_variables.analitic_jacobian_p'*(zeta_1-zeta_0)/Tmax;
    l_d_analytic(:,i) = cdpr_variables.analitic_jacobian_l'*(zeta_1-zeta_0)/Tmax;

    %update old values for differentiation
    zeta_old = zeta(:,i);
    for j = 1:cdpr_parameters.n_cables
        sigma_old(j) = cdpr_variables.cable(j).swivel_ang;
        psi_old(j) = cdpr_variables.cable(j).tan_ang; 
        l_old(j) = cdpr_variables.cable(j).complete_length; 
    end
end

% figure()
% subplot(2,1,1)
% plot(t,zeta,'LineWidth',1.5);
% legend('x','y','z','phi','psi','theta')
% subplot(2,1,2)
% plot(t,zeta_d_numeric,'LineWidth',1.5);
% legend('x_d','y_d','z_d','phi_d','psi_d','theta_d')

figure()
subplot(3,1,1)
plot(t(2:end),sigma_d_numeric(:,2:end),'LineWidth',1.5);
title('numeric')
subplot(3,1,2)
plot(t(2:end),sigma_d_analytic(:,2:end),'LineWidth',1.5);
title('analytic')
subplot(3,1,3)
plot(t(2:end),sigma_d_analytic(:,2:end)-sigma_d_numeric(:,2:end),'LineWidth',1.5);
title('difference')

figure()
subplot(3,1,1)
plot(t(2:end),psi_d_numeric(:,2:end),'LineWidth',1.5);
title('numeric')
subplot(3,1,2)
plot(t(2:end),psi_d_analytic(:,2:end),'LineWidth',1.5);
title('analytic')
subplot(3,1,3)
plot(t(2:end),psi_d_analytic(:,2:end)-psi_d_numeric(:,2:end),'LineWidth',1.5);
title('difference')

figure()
subplot(3,1,1)
plot(t(2:end),l_d_numeric(:,2:end),'LineWidth',1.5);
title('numeric');
subplot(3,1,2)
plot(t(2:end),l_d_analytic(:,2:end),'LineWidth',1.5);
title('analytic');
subplot(3,1,3)
plot(t(2:end),l_d_analytic(:,2:end)-l_d_numeric(:,2:end),'LineWidth',1.5);
title('difference');
end