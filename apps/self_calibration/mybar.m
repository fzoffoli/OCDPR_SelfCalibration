function [theta_m,J_theta] = mybar(cable_p,l)

fun = @(theta_motor)l-foo(cable_p,theta_motor);
theta_m = fsolve(fun,0);

if nargout>1
    [~,J_l] = foo(cable_p,theta_m);
    J_theta = 1/J_l;
end
end