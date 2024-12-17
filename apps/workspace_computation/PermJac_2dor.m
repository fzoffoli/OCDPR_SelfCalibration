function [Jc,Jd,P] = PermJac_2dor(J_T,i,j)

pi_fun=1:8;
if i<j
    pi_fun(j)=[];
    pi_fun(i)=[];
    pi_fun(7:8)=[i,j];
else
    pi_fun(i)=[];
    pi_fun(j)=[];
    pi_fun(7:8)=[j,i];
end
P=zeros(8);
for k=1:8
    P(k,pi_fun(k))=1;
end

Jp=J_T*P';
Jd=Jp(:,1:6);
Jc=Jp(:,7:8);
end