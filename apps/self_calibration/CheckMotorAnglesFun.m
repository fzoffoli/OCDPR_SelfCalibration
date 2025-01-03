function [fun,grad] = CheckMotorAnglesFun(cdpr_v,cdpr_p,pose,motor_angles)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_p,cdpr_v);
for i=1:cdpr_p.n_cables
    motor_angles_computation(i) = cdpr_v.cable(i).theta_motor;
    motor_angles_deriv(i) = cdpr_v.cable(i).l_theta_deriv;
end

fun = motor_angles_computation-motor_angles;

grad = diag(motor_angles_deriv)*cdpr_v.analitic_jacobian_l';
end