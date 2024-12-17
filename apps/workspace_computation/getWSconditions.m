% This function check the belonging of the given CDPR pose to the Wrench
% Feasible Error Insensitive Workspace giving a boolean in output.
% INPUT parameters and variables of the CDPR, position and orientation to
% be checked, ws_info, output data straucture, cables force-controlled,
% record structure.
% OUTPUT output data structure that contains the flag

function out = getWSconditions(cdpr_p,cdpr_v,position,orient,ws_info,ut,out,CablesForceControlled,rec)

% check reachable workspace belonging
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);
ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat.*1.1 lim.xyz_mean+lim.dl_frame-lim.dl_plat.*1.1];
if all(position>ws_limits(:,1)) && all(position<ws_limits(:,2))
    
    % check teiw belonging
    cdpr_v = UpdateIKZeroOrd(position,orient,cdpr_p,cdpr_v);
    out = WFworkspace_2dor(cdpr_p,cdpr_v,ut,[position;orient],ws_info.tension_limits,ws_info,out,CablesForceControlled,rec);


    % check accuracy limits
    if out.flag
        sigma_threshold=[5 0.5];      % mm/mm and deg/mm
        out.rotCardouSens(end)=180/pi*(out.rotCardouSens(end))/1000;
        if (out.posCardouSens(end)<sigma_threshold(1) && out.rotCardouSens(end)<sigma_threshold(2))
            out.flag=1;
        else
            out.flag=0;
        end
    end

else
    out.flag = 0;
end

end