function [findex,In1,Mult,comb,sigma,coM,Heurfindex,Heursigma]=sensitivity_2dor(sens,cdpr_p,cdpr_v)
% function [findex,In1,Mult,comb,sigma]=sensitivity_2dor(Jt,S)
% SENSITIVITY computes the force-distribution to one cable tension error
% INPUT: 
% Jt= transpose of jacobian matrix (Ngdl x Ncables), 
% S= geometric and inertial data
% OUTPUT: 
% findex = 1x2 array containing the pair of indices of the
% lowest-sensitivity cables
% In1 = matrix whose columns represent the sensitivity values of all possible
% cable pairs
% Mult = multeplicity, i.e. number of pairs with equal sensitivity (within a
% prescribed tolerance)
% comb = number of cable combinations
% sigma = minumum FD sensitivty
% coM = matrix of combinations
%
comb = nchoosek(cdpr_p.n_cables,cdpr_p.n_cables-cdpr_p.pose_dim);
co = (1:1:comb)';
coM=zeros(cdpr_p.n_cables,cdpr_p.n_cables);
m=1;
for ik=1:cdpr_p.n_cables
    for jk=ik+1:cdpr_p.n_cables
        coM(ik,jk)=co(m);
        m=m+1;
    end
end
In=zeros(comb, 2);
rr=0;
Jc = sens.Jc;
Jd = sens.Jd;
% if S.R==0
%     [Jc,Jd]=PermuteJ_2DOR(Jt,3,4);
% else
%     [Jc,Jd]=PermuteJ_2DOR(Jt,4,5);
% end
% SS1 = svd(Jd);
% if min(SS1)<exp(-8)
%     findex=[0,0];
% %     vv=0;
%     Mult=0;
% else
    v45 = [-linsolve(Jd,Jc); eye(2,2)];
    for i=1:(cdpr_p.n_cables-1)
        for j=(i+1):cdpr_p.n_cables
            rr=rr+1;
            v=[v45(i,:);v45(j,:)];
            
            if cond(v)>1000
               normj=0;
                
            else
                A=linsolve(v,eye(2));
                
                vv=v45*A;
                normj=norm(vv, inf);     
            end
                In(rr,1)=normj;
                
                  In(rr,2)=coM(i,j);
                
%                 In(rr,2)=i;
%                 In(rr,3)=j;
                
        end
        
    end
    
    In1=In(:,1);%[val of sigma for each combi   combi] (28x2)
%     In(rr+1:end,:)=[];

    sigma=min(In1(In1>0));%%m=min(In(:,1)(In(:,1)>0))
    ind=find(In1==sigma);
    i_Mult= find((In(:,1)<=sigma*1.010) & In(:,1)>0);%1.005
    if length(i_Mult)>1
        Mult=length(i_Mult);
    else
        Mult=1;
    end
    findex=In(ind(1),2);%solo ind?
%     comb=In(:,2:3);

%I took only the value I need 
% id=[1 5 6 7 10 11 12 21 26 28]'; %1.95

% id=[1 5 6 7  12 21 26 28]'; %2.187
% id=[5 7 10 11 12 21 26 28]';%2.0435
% id=[ 5 6  12 21 26 28]';%2.1874

% id=[2 13 14 15 17 19 20 24]';%  6.2934
id=[1 5 6 8 16 26 28]';%3.9455 heuristic combo

Heurind=[In(id) id];
Heurind1=Heurind(:,1);
Heursigma=min(Heurind1(Heurind1>0));
Heind=find(Heurind1==Heursigma);
Heurfindex=Heurind(Heind(1),2);
%
end
