function [norm_PT,J] = foo_calc_norm_PT(l_d,R,d_m,q,s,rho_m,h,theta_m)

if theta_m > l_d*2*pi/rho_m
    disp('theta motor too big!')
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