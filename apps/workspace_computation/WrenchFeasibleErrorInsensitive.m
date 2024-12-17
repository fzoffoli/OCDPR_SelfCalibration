%This function evaluates if a point inside the WFW is error insensitive,
%the algorthm for feasibility is described in Gouttefarde2015. This version
%of the Tension Sensitivity check reflects the worst case scenario and can
%be used without combination of errors (the result is the same).
%INPUT: cdpr param and variables, geometric parameters, length controlled 
%tension vector and its differential, nullspace basis of
%jacobian matrix and its differential, tension
%OUTPUT: feasibility or not of the point
function teiw = WrenchFeasibleErrorInsensitive(cdpr_p,ws_info,tau_p,tauP_dl,J_ort,dJ_ort,delta_tau_c,delta_l)

ii=7;
jj=8;
Im=[1,2,3,4,5,6,7,8];

N=J_ort+dJ_ort;
qmin=ws_info.tension_limits(1)*ones(cdpr_p.n_cables,1)-tau_p+...
    norm(tauP_dl,'inf').*abs(delta_l)+norm(J_ort,'inf')*abs(delta_tau_c(1))*ones(size(delta_l));
qmax=ws_info.tension_limits(2)*ones(cdpr_p.n_cables,1)-tau_p-...
    norm(tauP_dl,'inf').*abs(delta_l)-norm(J_ort,'inf')*abs(delta_tau_c(1))*ones(size(delta_l));
%determine first vertex vij intersecting two random lines Li and Lj
%FZ modified the initial value for the algorithm to let it converge faster
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

cnt = 0;
% pippo = [];
vertex=[];
while aa
    % determine the direction of motion
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
        disp('miao')
    end
    % determine the distance of motion
    alfa=zeros(1,cdpr_p.n_cables);    % COMMENTED FOR DEBUGGING
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
%     alfa=zeros(1,cdpr_p.n_cables);
%     for k=1:cdpr_p.n_cables
%         nk=N(k,:);
%         if nk*ni_p'>eps1
%             if (nk*fc-qmin(k)<0 && (abs(nk*fc-qmin(k))<eps1))%eps1
%                 alfa(k)=(qmin(k)-nk*fc)/(nk*ni_p');
%             elseif (((nk*fc-qmin(k)>0) && (nk*fc-qmax(k)<0)) || (abs(nk*fc-qmax(k))<eps1))%eps1
%                 alfa(k)=(qmax(k)-nk*fc)/(nk*ni_p');
%             else
%                 alfa(k)=10^(16);
%             end
%         elseif ((nk*ni_p'<0)  &&  (abs(nk*ni_p')<eps1)) %%nk*ni_p'<-eps1
%             if (nk*fc-qmax(k)>0 && (abs(nk*fc-qmax(k))<eps1))
%                 alfa(k)=(qmax(k)-nk*fc)/(nk*ni_p');
%             elseif (((nk*fc-qmin(k)>0) && (nk*fc-qmax(k)<0))|| (abs(nk*fc-qmin(k))<eps1))
%                 alfa(k)=(qmin(k)-nk*fc)/(nk*ni_p');
%             else
%                 alfa(k)=10^(16);
%             end
%         else %considered lines are parallel
%             if k==ii
%                 alfa(k)=0;
%             else
%                 alfa(k)=10^(16);
%             end
%         end
%     end
    % choose the first intersection point
    ralfa=round(alfa,6);
    [al,~]=min(alfa(ralfa>0));
    if isempty(al)
    else
        fc=fc+al*ni_p';
        ll=find(alfa==al);
        vertex(:,end+1)=fc;
    end
    if (qmax(8)<=qmin(8))    % heavy infeasibility condition added by FZ
        aa=0;
        teiw=0;
        break
    end
    if isempty(find(In==ll, ll))
        In(ll)=ll;
        vf=fc;
        jj=ii;
        ii=ll;
        aa=1;
    else
%         pippo(end+1)= norm(fc-vf);
%         if norm(fc-vf)>eps1%%
        if norm(fc-vf)>eps1%%
            jj=ii;
            ii=ll;
            aa=1;

        else
            if In(1)==Im(1) && In(2)==Im(2) && In(3)==Im(3)&& In(4)==Im(4) && In(5)==Im(5)...
                    && In(6)==Im(6) && In(7)==Im(7) && In(8)==Im(8)
                %  msgbox('polygon determined')
                aa=0;
                teiw=1;
            else
                %  msgbox('polygon inexistent')
                aa=0;
                teiw=0;
            end
        end
    end
    cnt=cnt+1;
    for p=1:6
        upper_lim(:,p)=linsolve([N(p,:);N(8,:)],[qmax(p);qmax(8)]);
        lower_lim(:,p)=linsolve([N(p,:);N(8,:)],[qmin(p);qmin(8)]);
    end
    delta=[max(upper_lim(1,:))-min(upper_lim(1,:)) max(lower_lim(1,:))-min(lower_lim(1,:))];
    if (cnt==1000 || (min(delta)<0.01 && rcond(N'*N)<1e-12) || rcond(N'*N)<1e-14)
        aa=0;
        teiw=0;
%         figure()
%         x=linspace(-2000,15000,1000);
%         for bb=1:8
%             ni=N(bb,:);
%             Limax=Lineq(x,-ni(1)/ni(2),qmax(bb)/ni(2));
%             Limin=Lineq(x,-ni(1)/ni(2),qmin(bb)/ni(2));
%             if bb==1
%                 plot(x,Limax,'r',x,Limin,'r')
%                 axis equal
%                 hold on
%             elseif bb==2
%                 plot(x,Limax,'b',x,Limin,'b')
%                 hold on
%             elseif bb==3
%                 plot(x,Limax,'g',x,Limin,'g')
%                 hold on
%             elseif bb==4
%                 plot(x,Limax,'m',x,Limin,'m')
%                 hold on
%             elseif bb==5
%                 plot(x,Limax,'c',x,Limin,'c')
%                 hold on
%             elseif bb==6
%                 plot(x,Limax,'y',x,Limin,'y')
%                 hold on
%             elseif bb==7
%                 plot(x,qmax(7)*ones(length(x),1),'k',x,qmin(7)*ones(length(x),1),'k')
%                 hold on
%             else
%                 plot(qmax(7)*ones(length(x),1),x,'k',qmin(7)*ones(length(x),1),x,'k')
%                 hold on
%             end
%         end
%         plot(vertex(1,:),vertex(2,:),'o')
    end
end
end