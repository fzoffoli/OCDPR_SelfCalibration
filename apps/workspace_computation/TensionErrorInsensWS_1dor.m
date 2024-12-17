function teiw=TensionErrorInsensWS_1dor(cdpr_p,ws_info,fp,tau_P_dl,J_ort,dJ_ort,delta_tau_c,delta_l)

tau_p = fp + tau_P_dl*delta_l+J_ort*delta_tau_c;
a=ws_info.tension_limits(2)*ones(cdpr_p.n_cables,1)-tau_p;    % equivalent of qmax
b=ws_info.tension_limits(1)*ones(cdpr_p.n_cables,1)-tau_p;    % equivalent of qmin
N = J_ort+dJ_ort;
minT=zeros(cdpr_p.n_cables,1);
maxT=minT;
for i=1:cdpr_p.n_cables
    if N(i)>0
        minT(i)=b(i)/N(i);
        maxT(i)=a(i)/N(i);
    else
        minT(i)=a(i)/N(i);
        maxT(i)=b(i)/N(i);
    end
end

tau_c_max=min(maxT);
tau_c_min=max(minT);
if tau_c_max>ws_info.tension_limits(2)
    tau_c_max=ws_info.tension_limits(2);
else
end
if tau_c_min<ws_info.tension_limits(1)
    tau_c_min=ws_info.tension_limits(1);
else
end
if tau_c_min<tau_c_max
    teiw=1;
else 
    teiw=0;
end