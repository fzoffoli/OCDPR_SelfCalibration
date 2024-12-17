% This function compute the cardou kinematic sensitivities described in 10.1109/TRO.2009.2037252
% INPUT Analytic Jacobian matrix used in direct kinematics of first order
% OUTPUT rotational and positional sensitivities
function [sigmaRot, sigmaPos, out]=SensCardou(K,q,out)
opt = optimoptions('linprog','Algorithm','interior-point-legacy','Display','off');

%%% brutal
if length(K)==6   
    J=linsolve(K,eye(6));
    Jp=J(1:3,:);
    Jr=J(4:6,:);
else 
    J=linsolve(K,eye(3));
    Jp=J(1:2,:);
    Jr=J(3,:);
end
sigmaRot=norm(Jr,'inf');
sigmaPos=norm(Jp,'inf');

% if q=="inf"
%     L=[K; -K];
%     b=ones(length(L),1);
%     
%     F=eye(length(K));
%     for i=1:length(F)
%         [x(:,i), fval(i), eflag(i), ~]=linprog(-F(:,i),L,b,[],[],[],[],opt); %%% Minus is due to the argument of linprog that finds a min, here we want the max of F*x
%     end
% 
%     if eflag==ones(length(F),1)
%         if length(fval)>3 %%% is the cable robot spatial?
%             sigmaPos=max(-fval(1:3)); %%% max(scalarFun)=-min(-scalarFun)
%             sigmaRot=max(-fval(4:6));
%         else
%             sigmaPos=max(-fval(1:2)); %%% max(scalarFun)=-min(-scalarFun)
%             sigmaRot=-fval(3);
%         end
%     else
%         sigmaRot=NaN;
%         sigmaPos=NaN;
%         fprintf('Invalid linear program solution \n');
%     end
% elseif q=="two"
%     if size(K,2)>3  %%% is the cable robot spatial?
%         Kp=K(:,1:3);
%         Kr=K(:,4:6);
%     else
%         Kp=K(:,1:2);
%         Kr=K(:,3);
%     end
%     Pp=eye(length(K))-Kp*((Kp'*Kp)\eye(2))*Kp';
%     Pr=eye(length(K))-Kr*((Kr'*Kr)^-1)*Kr';
%     sigmaPos=sqrt(norm((Kr'*Pp*Kr)\eye(2)));
%     sigmaRot=sqrt(norm((Kp'*Pr*Kp)^-1));
% else
%     sigmaRot=NaN;
%     sigmaPos=NaN;
%     fprintf('Invalid function argument \n');
% end
end