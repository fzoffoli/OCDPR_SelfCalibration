function JacobiansCheck(cdpr_parameters,cdpr_variables)

dt = 0.001;
Tmax = 5;
t = 0:dt:Tmax;
zeta_0 = [-0.5;0;0.5;0;0;0];
zeta_1 = [0.5;0;-0.5;0;pi/6;0];

for i = 1:length(t)
    zeta(:,i) = zeta_0+(zeta_1-zeta_0)*t(i)/Tmax;
    cdpr_variables = UpdateIKZeroOrd(zeta(1:3,i),zeta(4:6,i),cdpr_parameters,cdpr_variables);
    for j = 1:cdpr_parameters.n_cables
        sigma(j) = cdpr_variables.cable(j).swivel_ang;
        psi(j) = cdpr_variables.cable(j).tan_ang;
        l(j) = cdpr_variables.cable(j).complete_length;
    end

    [flag_s,err_s] = checkGradients(@(pose)CheckSwivelFun(cdpr_variables,cdpr_parameters,pose,sigma),zeta(:,i));
    [flag_p,err_p] = checkGradients(@(pose)CheckTangencyFun(cdpr_variables,cdpr_parameters,pose,psi),zeta(:,i));
    [flag_l,err_l] = checkGradients(@(pose)CheckCableLenghtsFun(cdpr_variables,cdpr_parameters,pose,l),zeta(:,i));
    
end

assert(flag_s,'Swivel Jacobian is wrong');
assert(flag_p,'Tangency jacobian is wrong');
assert(flag_l,'Cable length jacobian is wrong');

end