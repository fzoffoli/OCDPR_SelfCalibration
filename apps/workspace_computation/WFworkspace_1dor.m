function out = WFworkspace_1dor(cdpr_p,cdpr_v,ut,pose,tau_lim,ws_info,out,varargin)
%ws= WFworkspace_2dor(Jt,S)
%start

ii=7;
jj=8;
Im=[1,2,3,4,5,6,7,8];
iii=1;

for iii=1:7
    Jd = cdpr_v.geometric_jacobian;
    Jd(:,iii) = [];
    tempp(iii) = cond(Jd);
end

[~,ii] = min(tempp);
Jd = cdpr_v.geometric_jacobian;
Jd(:,ii) = [];
Jc = cdpr_v.geometric_jacobian(:,ii);

sens.index = ii;
sens.Jc= Jc;
sens.Jd = Jd;
%varargin{1}.SetFrame(cdpr_v,cdpr_p);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
fp=[linsolve(Jd,cdpr_v.platform.ext_load)];
a=-ws_info.tension_limits(2)*ones(cdpr_p.n_cables-1,1)+fp;
b=-ws_info.tension_limits(1)*ones(cdpr_p.n_cables-1,1)+fp;
coeff = linsolve(Jd,Jc);
minT=zeros(1,6);
maxT=minT;
for i=1:6
    if coeff>0
        minT(i)=a(i)/coeff(i);
        maxT(i)=b(i)/coeff(i);
    else
        minT(i)=b(i)/coeff(i);
        maxT(i)=a(i)/coeff(i);
    end
end

tau_max=min(maxT);
tau_min=max(minT);
if tau_max>ws_info.tension_limits(2)
    tau_max=ws_info.tension_limits(2);
else
end
if tau_min<ws_info.tension_limits(1)
    tau_min=ws_info.tension_limits(1);
else
end
if tau_min<tau_max
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
    out.mul(1,out.counter) = length(index);
    out.index(:,out.counter) = index(1);%cable number *************here
        out.WS_perf(1,out.counter) = index1;% sensitivity index
        out.sigma(1,out.counter) = index1(1);% sensitivity index
end