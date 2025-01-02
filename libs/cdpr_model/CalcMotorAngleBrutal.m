function [theta_motor,J] = CalcMotorAngleBrutal(R,d_m,q,s,rho_m,h,C,l)

a = (d_m^2/4+rho_m^2/(4*pi^2))^2;
b = 2*(d_m^2/4+rho_m^2/(4*pi^2))*(-d_m*(l+C)-q*rho_m/pi);
c = (-d_m*(l+C)-q*rho_m/pi)^2+2*(d_m^2/4+rho_m^2/(4*pi^2))*((l+C)^2-h^2-R^2+q^2-(s+d_m/2)^2)+(s+d_m/2)^2*rho_m^2/pi^2;
d = 2*(-d_m*(l+C)-q*rho_m/pi)*((l+C)^2-h^2-R^2+q^2-(s+d_m/2)^2)-(s+d_m/2)^2*(4*rho_m*q/pi);
e = ((l+C)^2-h^2-R^2+q^2-(s+d_m/2)^2)^2-(s+d_m/2)^2*(4*R^2-4*q^2);

Delta = 256*a^3*e^3-192*a^2*b*d*e^2-128*a^2*c^2*e^2+144*a^2*c*d^2*e-27*a^2*d^4+...
    144*a*b^2*c*e^2-6*a*b^2*d^2*e-80*a*b*c^2*d*e+18*a*b*c*d^3+16*a*c^4*e-...
    4*a*c^3*d^2-27*b^4*e^2+18*b^3*c*d*e-4*b^3*d^3-4*b^2*c^3*e+b^2*c^2*d^2;
P_ = 8*a*c-3*b^2;
R_ = b^3-4*a*b*c+8*a^2*d;
D_ = 64*a^3*e-16*a^2*c^2+16*a*b^2*c-16*a^2*b*d-3*b^4;

Delta_1 = 2*c^3-9*b*c*d+27*b^2*e+27*a*d^2-72*a*c*e;
Delta_0 = c^2-3*b*d+12*a*e;
p_ = (8*a*c-3*b^2)/(8*a^2);
q_ = (b^3-4*a*b*c+8*a^2*d)/(8*a^3);
Q_ = ((Delta_1+sqrt(Delta_1^2-4*Delta_0^3))/2)^(1/3);
S_ = 0.5*sqrt(-2/3*p_+1/(3*a)*(Q_+Delta_0/Q_));

x_1 = -b/(4*a)-S_+0.5*sqrt(-4*S_^2-2*p_+q_/S_);
x_2 = -b/(4*a)-S_-0.5*sqrt(-4*S_^2-2*p_+q_/S_);
x_3 = -b/(4*a)+S_+0.5*sqrt(-4*S_^2-2*p_-q_/S_);
x_4 = -b/(4*a)+S_-0.5*sqrt(-4*S_^2-2*p_-q_/S_);

% theta_motor = -b/(4*a);
theta_motor = real(x_1);

if nargout>1
    beta = asin(q/R-(theta_motor*rho_m)/(2*pi*R));
    point_P = [-R*sin(beta); R*cos(beta); h];
    point_T = [-R*sin(beta); s+d_m/2; 0];
    lambda = norm(point_P-point_T);

    A = R*cos(asin(q/R-(theta_motor*rho_m)/(2*pi*R)))-s-d_m/2;
    B = (q-theta_motor*rho_m/(2*pi))*rho_m/(2*pi*R);
    C_ = lambda*sqrt(1-(q/R-(theta_motor*rho_m)/(2*pi*R))^2);

    J_lambda = A*B/C_; 
    J = 1/(J_lambda + d_m/2);
end
end