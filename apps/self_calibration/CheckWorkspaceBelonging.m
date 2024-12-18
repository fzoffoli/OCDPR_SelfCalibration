function flag = CheckWorkspaceBelonging(cdpr_p,cdpr_v,ws_info)
% This function determines the wrench feasibility of the input pose using
% the algorithm described in Gouttefarde2015. The cable-to-cable and 
% cable-to-platform interference is also checked using Perreault2010.

ii=7;
jj=8;
Im=[1,2,3,4,5,6,7,8];

[Jc,Jd,~]=PermJac_2dor(cdpr_v.geometric_jacobian_l,ii,jj);

%%%% rows of the starting point of the algorithm must be optimized
ii=7;
jj=8;

%compute N and fp
N=[-linsolve(Jd,Jc); eye(2,2)];
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
fp=[linsolve(Jd,cdpr_v.platform.ext_load);0;0];
qmax=ws_info.tension_limits(2)*ones(cdpr_p.n_cables,1)-fp;
qmin=ws_info.tension_limits(1)*ones(cdpr_p.n_cables,1)-fp;

%determine first vertex vij intersecting two random lines Li and Lj
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
        elseif ((nk*ni_p'<0)  &&  (abs(nk*ni_p')>eps1))
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
        %         fc=fc;
        %         i=j;
        %         j=i;
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
        if norm(fc-vf)>eps1
            jj=ii;
            ii=ll;
            aa=1;
        else
            if In(1)==Im(1) && In(2)==Im(2) && In(3)==Im(3)&& In(4)==Im(4) && In(5)==Im(5)...
                    && In(6)==Im(6) && In(7)==Im(7) && In(8)==Im(8) && ~CableInterfPerreault(cdpr_v,cdpr_p)
                %  msgbox('polygon determined')
                aa=0;
                vertex( :, all(~vertex,1) ) = [];
                flag = 1;
            else
                %  msgbox('polygon inexistent')
                aa=0;
                flag = 0;
            end
        end
    end
    cnt=cnt+1;
    for p=1:6
        upper_lim(:,p)=linsolve([N(p,:);N(8,:)],[qmax(p);qmax(8)]);
        lower_lim(:,p)=linsolve([N(p,:);N(8,:)],[qmin(p);qmin(8)]);
    end
    delta=[max(upper_lim(1,:))-min(upper_lim(1,:)) max(lower_lim(1,:))-min(lower_lim(1,:))];
    if (cnt==1000)
        aa=0;
        flag = 0;
    end
end
end
