% This boolean function determines the belonging to the ws of interest
% INPUT position (hip constant orientation) cdpr variables and parameters,
% info on tension limits couple of cables force controlled
% OUTPUT 1 if feasible 0 if not feasible

function [flag,out] = BoolWsCriterion(position,cdpr_p,cdpr_v,ws_info,cableIdx,out)

cdpr_v = UpdateIKZeroOrd(position,[0;0;0],cdpr_p,cdpr_v);

Im=[1,2,3,4,5,6,7,8];

iii=cableIdx(1);             %%% Here the choice of the force controlled cables is made before
jjj=cableIdx(2);

[Jc,Jd,~]=PermJac_2dor(cdpr_v.geometric_jacobian,iii,jjj);
sens.ii = iii;
sens.jj = jjj;
sens.Jc= Jc;
sens.Jd = Jd;

%%%% rows of the starting point of the algorithm must be optimized
ii=7;
jj=8;

%compute N and fp
N=[-linsolve(Jd,Jc); eye(2,2)];
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
fp=[linsolve(Jd,cdpr_v.platform.ext_load);0;0];
qmax=ws_info.tension_limits(2)*ones(cdpr_p.n_cables,1)-fp;
qmin=ws_info.tension_limits(1)*ones(cdpr_p.n_cables,1)-fp;

jj=8;
ni=N(ii,:);
nj=N(jj,:);


fc = linsolve([ni;nj],[qmin(ii);qmin(jj)]);
eps1 = 10^(-8);
eps2 = 10^(-5);
In=zeros(1,cdpr_p.n_cables);
for ij=1:cdpr_p.n_cables
    if ((abs(N(ij,:)*fc-qmin(ij))<eps2)||(abs(N(ij,:)*fc-qmax(ij))<eps2)||((N(ij,:)*fc-qmin(ij)>0)&&(N(ij,:)*fc-qmax(ij)<0)))%eps1
        In(ij)=ij;
    else
        In(ij)=0;
    end
end
vf=fc;
%follow line Li and compute ni_perp
aa=1;
cnt=0;
while aa

    ni=N(ii,:);
    nj=N(jj,:);
    ni_p1 = [ni(2) -ni(1)];
    ni_p2 = [-ni(2) ni(1)];
    if abs(nj*fc-qmin(jj))<eps2
        if nj*ni_p1'>=0
            ni_p=ni_p1;
        else
            ni_p=ni_p2;
        end
    elseif abs(nj*fc-qmax(jj))<eps2
        if nj*ni_p1'<=0
            ni_p=ni_p1;
        else
            ni_p=ni_p2;
        end
    else
        msgbox('miao')
    end
    alfa=zeros(1,cdpr_p.n_cables);
    for k=1:cdpr_p.n_cables
        nk=N(k,:);
        if nk*ni_p'>eps1
            if (nk*fc-qmin(k)<0 && ~(abs(nk*fc-qmin(k))<eps2))%eps1
                alfa(k)=(qmin(k)-nk*fc)/(nk*ni_p');
            elseif (((nk*fc-qmin(k)>0) && (nk*fc-qmax(k)<0)) || (abs(nk*fc-qmin(k))<eps2))%eps1
                alfa(k)=(qmax(k)-nk*fc)/(nk*ni_p');
            else
                alfa(k)=10^(16);
            end
        elseif ((nk*ni_p'<0)  &&  (abs(nk*ni_p')>eps1)) %%nk*ni_p'<-eps1
            if (nk*fc-qmax(k)>0 && ~(abs(nk*fc-qmax(k))<eps1))
                alfa(k)=(qmax(k)-nk*fc)/(nk*ni_p');
            elseif (((nk*fc-qmin(k)>0) && (nk*fc-qmax(k)<0))|| (abs(nk*fc-qmax(k))<eps1))
                alfa(k)=(qmin(k)-nk*fc)/(nk*ni_p');
            else
                alfa(k)=10^(16);
            end
        else %considered lines are parallel
            if k==ii
                alfa(k)=0;
            else

                alfa(k)=10^(16);
            end
        end
    end
    ralfa=round(alfa,6);
    [al,~]=min(alfa(ralfa>0));
    if isempty(al)
    else
        fc=fc+al*ni_p';
        ll=find(alfa==al);
    end
    if isempty(find(In==ll, ll))
        In(ll)=ll;
        vf=fc;
        jj=ii;
        ii=ll;
        aa=1;
        vertex=zeros(2,8);
    else
        vertex(:,ll)=fc;
    
        if norm(fc-vf)>eps1%%
            jj=ii;
            ii=ll;
            aa=1;
        else
            if In(1)==Im(1) && In(2)==Im(2) && In(3)==Im(3)&& In(4)==Im(4) && In(5)==Im(5)...
                    && In(6)==Im(6) && In(7)==Im(7) && In(8)==Im(8) && ~CableInterfPerrault(cdpr_v,cdpr_p)
                %  msgbox('polygon determined')
                aa=0;
                vertex( :, all(~vertex,1) ) = [];

                %%%% INTRODUCTION FZ SENSITIVITY INDEXES
                delta_l=0.01*ones(cdpr_p.n_cables,1);                       % cable length measure error
                delta_tau_c=5*ones(cdpr_p.n_cables-cdpr_p.pose_dim,1);      % cable tension measure error
                tau_c = TensDistributionMean(vertex);
                [~,tauP_dl,~,dJ_ort]=InputRatioIndex(cdpr_v,cdpr_p,cableIdx,Jd,Jc,tau_c',delta_l);
                teiw=TensionErrorInsensWS_2dor(cdpr_p,ws_info,fp,tauP_dl,N,dJ_ort,delta_tau_c,delta_l);
                out.counter=out.counter+1;
                [rotCardouSens, posCardouSens, out]=SensCardou(Jd','inf',out);
                
                rotCardouSens=180/pi*(rotCardouSens)/1000;  % um conversion

                %%% Accuracy set limits and fault tolerance evaluation
                sigma_threshold=[5 3];      % [mm/mm deg/mm]
                if teiw &&(posCardouSens<sigma_threshold(1) && rotCardouSens<sigma_threshold(2))
                    flag=1;
                else
                    flag=0;
                end
                %
            else
                %                                 msgbox('polygon inexistent')
                aa=0;
                flag=0;
            end
        end
    end
    cnt=cnt+1;
    if (cnt>=1000)
        aa=0;
        flag=0;
    end
end
end