function [fun, J] = CalcMotorAngle(cable_length,theta_motor,cable_p)
%CALCMOTORANGLE is the aux function for a nonlinear solver to determine the
% motor angle for a given cable length

[lambda,J_lambda] = CalcTransmissionLambda(cable_p.drum_length,cable_p.flag_radius,...
    cable_p.drum_diameter,cable_p.q,cable_p.s,cable_p.drum_pitch,...
    cable_p.h,theta_motor);
fun = cable_p.drum_diameter*theta_motor/2+lambda-cable_p.C-cable_length;

if nargout>1
    J = J_lambda + cable_p.drum_diameter/2;
end
end