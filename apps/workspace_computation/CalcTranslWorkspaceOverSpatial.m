function out = CalcTranslWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec,CablesForceControlled)

out.counter = 0;
out.full_counter =0;
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);

%change limits /2
ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat.*1.1 lim.xyz_mean+lim.dl_frame-lim.dl_plat.*1.1]; % /2
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe+0;
out.ws_volume=(max(ws_limits(1,:))-min(ws_limits(1,:)))*(max(ws_limits(2,:))-min(ws_limits(2,:)))*(max(ws_limits(3,:))-min(ws_limits(3,:)));
if mod(ws_info.mesh_divider,2)==0
  ws_info.mesh_divider = ws_info.mesh_divider-1;
end
ws_info.e_mesh = (ws_info.mesh_divider-1)/2;
orient = [0;pi/6;0];
for z = ws_limits(3,2):(ws_limits(3,1)-ws_limits(3,2))/ws_info.mesh_divider:ws_limits(3,1)
    for y = ws_limits(2,2):(ws_limits(2,1)-ws_limits(2,2))/ws_info.mesh_divider:ws_limits(2,1)
        for x = ws_limits(1,2):(ws_limits(1,1)-ws_limits(1,2))/ws_info.mesh_divider:ws_limits(1,1)%%

            cdpr_v = UpdateIKZeroOrd([x;y;z],orient,cdpr_p,cdpr_v);
            out.full_counter=out.full_counter+1;

            if (cdpr_p.n_cables-cdpr_p.pose_dim==2)
                out = WFworkspace_2dor(cdpr_p,cdpr_v,ut,[x;y;z;orient],ws_info.tension_limits,ws_info,out,CablesForceControlled,rec);
            else
                out = WFworkspace_1dor(cdpr_p,cdpr_v,ut,[x;y;z;orient],ws_info.tension_limits,ws_info,out,rec);% hereeeeee****************************
            end
        end 
    end 
end

end