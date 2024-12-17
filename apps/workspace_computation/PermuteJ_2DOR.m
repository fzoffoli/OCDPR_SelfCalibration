function [Jc,Jd]=PermuteJ_2DOR(Jt,i,j)
%partition of jacobian matrix considering force controlled (c) and length
%controlled (d) variables. It works for 2 degrees of redundancy.
%INPUT:
%Jt=jacobian matrix (n dof x m cable)
%i,j= indeces of the force controlled cables
%OUTPUT:
%Jc=jcacopbian part for force-controlled variables (n dof x n dor)
%Jd=jacobian parte for length-controlled variables (n dof x n dof)
 Jd=Jt;
 Jd(:,j)=[];
 Jd(:,i)=[];
 Jc=[Jt(:,i) Jt(:,j)]; 