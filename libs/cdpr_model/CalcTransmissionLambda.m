function [norm_PT,J] = CalcTransmissionLambda(l_d,R,d_m,q,s,rho_m,h,theta_m)
%CALCTRANSMISSIONLAMBDA compute the cable lenght variation due to flag movement. 
%INPUTS:  
%   l_d is the drum length
%   R is the flag radius
%   d_m is the drum diameter
%   q is the x coordinate of the flag axis with respect to the drum frame
%   s is the y coordinate of the flag axis with respect to the drum frame
%   rho_m is the drum pitch
%   h is the z coordinate of the flag radius with respect to the drum frame
%   theta_m is the motor angle
%OUTPUTS:
%   variable segment norm and its jacobian with respect to theta_m
if theta_m > l_d*2*pi/rho_m
    msgbox('theta motor too big!')
end
beta = asin(q/R-(theta_m*rho_m)/(2*pi*R));
P = [-R*sin(beta); R*cos(beta); h];
T = [-R*sin(beta); s+d_m/2; 0];
norm_PT = norm(P-T);

if nargout > 1
    A = R*cos(asin(q/R-(theta_m*rho_m)/(2*pi*R)))-s-d_m/2;
    B = (q-theta_m*rho_m/(2*pi))*rho_m/(2*pi*R);
    C = norm_PT*sqrt(1-(q/R-(theta_m*rho_m)/(2*pi*R))^2);

    J = A*B/C; 
end

end

