function vol_inst = CalcVolInstall(cdpr_parameters,ws_info)
lim = DetermineLimits(cdpr_parameters,ws_info.z_inferior_limit);
ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat.*1.1 lim.xyz_mean+lim.dl_frame-lim.dl_plat.*1.1];
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe+0;
UpperBound=ws_limits(:,2)';
LowerBound=ws_limits(:,1)';
InstallationBounds(1,:)=LowerBound;
InstallationBounds(2,:)=[LowerBound(1:2) UpperBound(3)];
InstallationBounds(3,:)=[LowerBound(1) UpperBound(2) LowerBound(3)];
InstallationBounds(4,:)=[UpperBound(1) LowerBound(2:3)];
InstallationBounds(5,:)=[LowerBound(1) UpperBound(2:3)];
InstallationBounds(6,:)=[UpperBound(1) LowerBound(2) UpperBound(3)];
InstallationBounds(7,:)=[UpperBound(1:2) LowerBound(3)];
InstallationBounds(8,:)=UpperBound;
[~,vol_inst]=convhull(InstallationBounds);
end