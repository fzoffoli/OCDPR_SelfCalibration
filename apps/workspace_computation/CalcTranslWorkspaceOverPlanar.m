function out= CalcTranslWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec,CablesForceControlled)

out.counter = 0;
out.full_counter = 0;

ws_limits_x = linspace(-cdpr_p.cable(1,1).pos_OD_glob(1)+ ...
    2*abs(cdpr_p.cable(1,1).pos_PA_loc(1)), ...
    cdpr_p.cable(1,1).pos_OD_glob(1)- ...
    2*abs(cdpr_p.cable(1,1).pos_PA_loc(1)),ws_info.mesh_divider);

ws_limits_z = linspace(-cdpr_p.cable(1,1).pos_OD_glob(3)+ ...
    2*abs(cdpr_p.cable(1,1).pos_PA_loc(3)), ...
    cdpr_p.cable(1,1).pos_OD_glob(3)- ...
    2*abs(cdpr_p.cable(1,1).pos_PA_loc(3)),ws_info.mesh_divider);

out.ws_volume=(max(ws_limits_x)-min(ws_limits_x))*(max(ws_limits_z)-min(ws_limits_z));

orient = [0;0;0];
y=0;
for z = 1:ws_info.mesh_divider
    for x = 1:ws_info.mesh_divider
        %             x=2.58; y= 0.345; z= 1.1634;
        cdpr_v = UpdateIKZeroOrd([ws_limits_x(x);y;ws_limits_z(z)],orient,cdpr_p,cdpr_v);
        %             if (((x<=0) && (y<=0 )) || ((x>0) && (y<0 ) && (z<0 )) || ((x<=0) && (y>0 )) || ((x>0) && (y>0 ) ))

        out.full_counter = out.full_counter+1;
        if (cdpr_p.n_cables-cdpr_p.pose_dim==2)
            out = WFworkspace_2dor(cdpr_p,cdpr_v,ut,[x;y;z;orient],ws_info.tension_limits,ws_info,out,CablesForceControlled,rec);
        else
            out = WFworkspace_1dor_planar(cdpr_p,cdpr_v,ut,[x;y;z;orient],ws_info.tension_limits,ws_info,out,CablesForceControlled,rec);% hereeeeee****************************
        end
        %rec.SetFrame(cdpr_v,cdpr_p);


        %             else%
        %             end %
    end
end

end