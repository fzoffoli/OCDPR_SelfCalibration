function out = CalcReachWorkspaceUnderSpatialMod(cdpr_p,cdpr_v,ut,ws_info,rec)

out.counter = 0;
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);


%ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat lim.xyz_mean+lim.dl_frame-lim.dl_plat];
ws_limits = [lim.xyz_mean-lim.dl_frame lim.xyz_mean+lim.dl_frame];
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe-0.5;
% ws_limits(:,1) = [-2.7;-1.9;-4.5];
% ws_limits(:,2) = [2.7;1.9;-2.5];
ws_limits = [ws_limits;-pi/18 pi/18; -pi/18 pi/18; -pi/18 pi/18];
if mod(ws_info.mesh_divider,2)==0
  ws_info.mesh_divider = ws_info.mesh_divider-1;
end
ws_info.e_mesh = (ws_info.mesh_divider-1)/2;

%iter = zeros(cdpr_p.n_cables,1); iter(cdpr_p.n_cables) = -1;
iter = zeros(3,1); iter(3) = -1;
dir = ones(3,1);
i = 3;
res = [];

ctr = (ws_limits(:,2)+ws_limits(:,1))/2;
del = (ws_limits(:,2)-ws_limits(:,1))/(ws_info.mesh_divider-1);
%del = cdpr_p.underactuated_platform.permutation_matrix*del;
del(4:end) = [];

ws_info.limits = ws_limits;
ctr_p = ctr;
contr_ctr = ctr_p(1:3);
contr = contr_ctr;
free = ctr_p(4:end);
ig_s = zeros(3,3);

while 1
  iter(i) = iter(i)+1;
  contr(i) = contr_ctr(i)+dir(i)*iter(i)*del(i);
  if iter(i)> ws_info.e_mesh
    free = ig_s(:,i);
    if dir(i) == 1
      dir(i) = -1;
      iter(i) = 0;
    else
      dir(i) = 1;
      iter(i) = -1;
      i = i-1;
      if i==0
        break
      end
    end
  else
    if i==3
      conf = cdpr_p.underactuated_platform.permutation_matrix'*[contr;free];
      if ws_info.visualize_cdpr
       % [conf,out] = Solve_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out,rec);
        [conf,out] = Opadtimize_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out,rec);
      else
        %[conf,out] = Solve_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out);
        [conf,out] = Optimize_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out,rec);
      end
      conf_P = conf(4:6);
      free = conf_P;
      if (iter(i)==0 && dir(i)==1)
        ig_s(:,i) = free;
      end
    else
      if (iter(i)==1 && dir(i)==1)
        ig_s(:,i) = ig_s(:,i+1);
      end
      i = i+1;
    end
  end
end

end