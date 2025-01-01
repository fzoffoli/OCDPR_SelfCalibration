function [l,J_l] = foo(cable_p,theta_motor)

[lambda,J_lambda] = CalcTransmissionLambda(cable_p.drum_length,cable_p.flag_radius,...
    cable_p.drum_diameter,cable_p.q,cable_p.s,cable_p.drum_pitch,...
    cable_p.h,theta_motor);
l = theta_motor*cable_p.drum_diameter/2+lambda-cable_p.C;

if nargout>1
    J_l = cable_p.drum_diameter/2+J_lambda;
end
end