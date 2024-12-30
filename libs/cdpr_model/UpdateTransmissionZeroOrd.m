function cable_v = UpdateTransmissionZeroOrd(cable_p,cable_v)
%UPDATETRANSMISSIONZEROORD updates the transmission variables for a given
%cable length

[cable_v.theta_motor] = fsolve(@(theta_motor)CalcMotorAngle(cable_v.complete_length,theta_motor,cable_p));
[cable_v.lambda,J_lambda] = CalcTransmissionLambda(cable_p.drum_length,cable_p.flag_radius,...
    cable_p.drum_diameter,cable_p.q,cable_p.s,cable_p.drum_pitch,...
    cable_p.h,theta_motor);
cable_v.l_theta_motor_deriv = J_lambda + cable_p.drum_diameter/2;

end