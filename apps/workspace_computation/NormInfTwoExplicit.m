function [posSens, rotSens] = NormInfTwoExplicit(J_d)

options=optimoptions(@fmincon,'MaxIterations',4000);
options2=optimoptions(@fminunc,'Display','off');;

J=(J_d'\eye(6));
eps_rot=1E-3*ones(6,1);
eps_pos=1E-6*ones(6,1);
A=[eye(6);-eye(6)];
b_rot=[-eps_rot;-eps_rot];
b_pos=[-eps_pos;-eps_pos];
l_d_dt_0=ones(6,1);

% fun_pos =  @(l_dt)-norm(J(1:3,:)*l_dt)/norm(l_dt,'inf');
% [x, fval, eflag]=fmincon(fun_pos,l_d_dt_0,A,b_pos);
% posSens=-fval;
% 
% fun_rot =  @(l_dt)-norm(J(4:6,:)*l_dt)/norm(l_dt,'inf');
% [x, fval, eflag]=fmincon(fun_rot,l_d_dt_0,A,b_rot);
% rotSens=-fval;

fun_pos =  @(l_dt)-norm(J(1:3,:)*l_dt)/norm(l_dt,'inf');
[x_p, fval_p, eflag_p]=fminunc(fun_pos,l_d_dt_0,options2);
if eflag_p>0 && norm(x_p,'inf')>1E-6
    posSens=-fval_p;
else
    posSens=0;
end

fun_rot =  @(l_dt)-norm(J(4:6,:)*l_dt)/norm(l_dt,'inf');
[x_r, fval_r, eflag_r]=fminunc(fun_rot,l_d_dt_0,options2);
if eflag_r>0 && norm(x_r,'inf')>1E-3
    rotSens=-fval_r;
else
    rotSens=0;
end

end