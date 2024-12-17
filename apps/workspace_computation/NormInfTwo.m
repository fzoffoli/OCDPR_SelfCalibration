function [posSens, rotSens] = NormInfTwo(J)
J_dk=(J')\eye(6);

options=optimoptions(@fmincon,'MaxIterations',30000,'MaxFunctionEvaluations',30000*6,'Display','off');

l_dt_guess=ones(6,1);
fun_pos=@(l_dt)-norm(J_dk(1:3,:)*l_dt);
[x_P,fval_p,eflag_P]=fmincon(fun_pos,l_dt_guess,[],[],[],[],[],[],@SensNonlinc,options);
if eflag_P>0
    posSens=-fval_p;
else
    posSens=NaN;
end

fun_rot=@(l_dt)-norm(J_dk(4:6,:)*l_dt);
[x_r,fval_r,eflag_r]=fmincon(fun_rot,l_dt_guess,[],[],[],[],[],[],@SensNonlinc,options);

if eflag_r>0
    rotSens=-fval_r;
else 
    rotSens=NaN;
end
end