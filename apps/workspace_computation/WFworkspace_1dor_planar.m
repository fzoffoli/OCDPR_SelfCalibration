function out=WFworkspace_1dor_planar(cdpr_p,cdpr_v,ut,pose,tau_lim,ws_info,out,cableIdx,varargin)

ii=cableIdx(1);
Im=[1,2,3,4];
ws_info.tension_limits(1)=20;
ws_info.tension_limits(2)=80;

J_planar=cdpr_v.geometric_jacobian;
J_planar(6,:)=[];
J_planar(4,:)=[];
J_planar(2,:)=[];
Jd = J_planar;
Jd(:,ii) = [];
Jc = J_planar(:,ii);

sens.index = ii;
sens.Jc= Jc;
sens.Jd = Jd;
%varargin{1}.SetFrame(cdpr_v,cdpr_p);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
ext_wrench=[0; cdpr_v.platform.ext_load(3); 0];
fp=[linsolve(Jd,ext_wrench);0];
a=ws_info.tension_limits(2)*ones(cdpr_p.n_cables,1)-fp;
b=ws_info.tension_limits(1)*ones(cdpr_p.n_cables,1)-fp;
coeff = [-linsolve(Jd,Jc); 1]; %%% this is the nullspace basis
minT=zeros(cdpr_p.n_cables,1);
maxT=minT;
for i=1:cdpr_p.n_cables
    if coeff(i)>0
        minT(i)=b(i)/coeff(i);
        maxT(i)=a(i)/coeff(i);
    else
        minT(i)=a(i)/coeff(i);
        maxT(i)=b(i)/coeff(i);
    end
end

tau_c_max=min(maxT);
tau_c_min=max(minT);
if tau_c_max>ws_info.tension_limits(2)
    tau_c_max=ws_info.tension_limits(1);
else
end
if tau_c_min<ws_info.tension_limits(1)
    tau_c_min=ws_info.tension_limits(2);
else
end
if tau_c_min<tau_c_max
    out.counter = out.counter+1;
    out.pose(:,out.counter) = cdpr_v.platform.pose;
    out.position(:,out.counter) = out.pose(1:3,out.counter);
    out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
    out.constr(1,out.counter) = 0;
    out.tension_vector(:,out.counter)= zeros(cdpr_p.n_cables,1);
    out.cable_length(:,out.counter) = cdpr_v.cable_vector;

    vv = -coeff;
    index1=norm(vv, inf);
    % [B,index1]=maxk(abs(vv),S.dor);
    if (index1<1.05)
        index= find(abs(vv)>0.95 & abs(vv)<1.05);
        index=[index ;sens.index];%%%%%%%%%%%%%%%%%%%%;
        %      index= find(abs(vv)==index1);
    else
        index= find(abs(vv)>index1*0.95 & abs(vv)<index1*1.05);
        vv = [vv;1];
        vv = vv./max(abs(vv));
        elm = find(abs(vv)<0.95);
        index1 = norm(vv(elm),inf);
        for k=1:length(index)
            if index(k)>=sens.index
                index(k) = index(k)+1;
            end

        end
    end
    index1=norm((Jd\eye(3)*Jc),'inf');
    out.mul(1,out.counter) = length(index);
    out.index(:,out.counter) = index(1);%cable number *************here
    out.WS_perf(1,out.counter) = index1;% sensitivity index
    out.sigma(1,out.counter) = index1(1);% sensitivity index
%     out.rotSensitivity(out.counter)=0;
%     out.posSensitivity(out.counter)=0;
    [out.rotCardouSens(out.counter),out.posCardouSens(out.counter)]=SensCardou(Jd','inf');
    
    %%% Tension-Error-Insensitive Workspace computation
    tau_c=mean([tau_c_min tau_c_max]);
    d_l = 0.005;
    d_tau_c=5;
    errComb = permn([1 -1],cdpr_p.n_cables)';
    for k = 1:length(errComb)
        err  = [errComb(1:3,k)*d_l; errComb(4,k)*d_tau_c];
        delta_l = [err(1:3); 0];
        delta_tau_c = err(end);
        [~, tauP_dl, ~, dJ_ort]=InputRatioIndex(cdpr_v,cdpr_p,cableIdx,Jd,Jc,tau_c,delta_l);
        out.teiw(out.counter)=TensionErrorInsensWS_1dor(cdpr_p,ws_info,fp,tauP_dl,coeff,dJ_ort,delta_tau_c,delta_l);
        if out.teiw == 0 
            break
        end
    end
end