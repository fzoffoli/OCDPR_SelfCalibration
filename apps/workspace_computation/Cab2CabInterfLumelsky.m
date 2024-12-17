% INPUT cable attach points
% OUTPUT flag of interference between cables
function interf_detected = Cab2CabInterfLumelsky(cdpr_v,cdpr_p)
    
    % check for all combinations of cables
    cableComb=nchoosek(1:cdpr_p.n_cables,cdpr_p.n_cables-cdpr_p.pose_dim);
    interf_detected=0;
    for cIdx=1:length(cableComb)
        cabCheck=cableComb(cIdx,:);

        % compute the square length of the segments and other params
        D1=0;
        D2=0;
        S1=0;
        S2=0;
        R=0;
        for i = 1:3
            di1 = cdpr_p.cable(cabCheck(1)).pos_OD_glob(i)-cdpr_v.cable(cabCheck(1)).pos_OA_glob(i);
            di2 = cdpr_p.cable(cabCheck(2)).pos_OD_glob(i)-cdpr_v.cable(cabCheck(2)).pos_OA_glob(i);
            di12 = cdpr_v.cable(cabCheck(2)).pos_OA_glob(i)-cdpr_v.cable(cabCheck(1)).pos_OA_glob(i);
            D1=D1+di1^2;
            D2=D2+di2^2;
            S1=S1+di1*di12;
            S2=S2+di2*di12;
            R=R+di1*di2;
        end

        % determine the position of lines intersection
        den=(D1*D2-R^2);
        eps = 1.0e-10;  % numeric zero
        if den<eps  % cables are parallel
            t=0;
        else
            t=(S1*D2-S2*R)/den;
            if t<0  % intersection outside segment
                t=0;
            elseif t>1
                t=1;
            end
        end
        u=(t*R-S2)/D2;
        if u<0      % intersection outside segment 
            u=0;
        elseif u>1
            u=1;
        end
        
        % calculate the minimum distance between segments
        minD=0;
        for i = 1:3
            di1 = cdpr_p.cable(cabCheck(1)).pos_OD_glob(i)-cdpr_v.cable(cabCheck(1)).pos_OA_glob(i);
            di2 = cdpr_p.cable(cabCheck(2)).pos_OD_glob(i)-cdpr_v.cable(cabCheck(2)).pos_OA_glob(i);
            di12 = cdpr_v.cable(cabCheck(2)).pos_OA_glob(i)-cdpr_v.cable(cabCheck(1)).pos_OA_glob(i);
            minD = minD+(di1*t-di2*u-di12)^2;
        end
        % check for interference
        limD=0.015; % cable diameter + SF
        if sqrt(minD)<limD
            interf_detected=1;
        end
    end
end