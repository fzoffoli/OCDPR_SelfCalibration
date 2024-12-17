%This function plot the convex hull of the HPFT workspace calculated as
%boundary
function PlotWorkspaceBound(sim_title,config_name,cdpr_parameters,cdpr_variables,ws_info,X,out)

%%% Selection of force controlled cables
cableCoupleIndex=X;
cableComb=nchoosek(1:cdpr_parameters.n_cables,cdpr_parameters.n_cables-cdpr_parameters.pose_dim);
cablesForceControlled=cableComb(cableCoupleIndex,:);

%%% Generation of robot architecture
rec = RecordType(cdpr_parameters,sim_title,config_name,cablesForceControlled);
try
    wp_name = strcat(config_name,'_WS.mat');
    load(wp_name);
    rec = rec.ResetFigureLimits(ws_data.limits,8);
catch
    ws_data = [];
end
% rec.SetFrame(cdpr_variables,cdpr_parameters);

%%% Adjust figure limits
set(gca,'linewidth',1.5)
lim = DetermineLimits(cdpr_parameters,ws_info.z_inferior_limit);
ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat.*1.1 lim.xyz_mean+lim.dl_frame-lim.dl_plat.*1.1]; % installation boundaries to be adjusted
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe+0;
UpperBound=ws_limits(:,2)+0.5*abs(ws_limits(:,2));
LowerBound=ws_limits(:,1)-0.5*abs(ws_limits(:,1));
rec.ResetFigureLimits([LowerBound UpperBound],ws_info.display_grid_divider);%%% be careful on erasing data

%%% Graphics
trisurf(out.k,out.BoundaryMatrix(:,1),out.BoundaryMatrix(:,2),out.BoundaryMatrix(:,3),'FaceColor',[128/255 0 128/255]);
% trisurf(out.k,out.BoundaryMatrix(:,1),out.BoundaryMatrix(:,2),out.BoundaryMatrix(:,3),'FaceColor',"#A2142F");
% title(strcat('HPFT workpsace ', num2str(cablesForceControlled)));
ax=gca;
ax.FontSize=12;

figure()
X_pie = [1-out.V out.V];
p=pie(X_pie);
p(1).FaceColor = [1 1 1];
p(2).FontSize = 16;
% p(3).FaceColor = [0.9769 0.9839 0.0805];
% p(4).FontSize = 16;
% p(3).FaceColor = [0.2422 0.1504 0.6603];
% p(4).FontSize = 16;
% p(3).FaceColor = [0.6350 0.0780 0.1840];
p(3).FaceColor = [128/255 0 128/255];
p(4).FontSize = 16;

%%% draw platform in old way
figure()
attach_p=zeros(cdpr_parameters.n_cables,3);
for i = 1:cdpr_parameters.n_cables
    attach_p(i,:) = cdpr_parameters.cable(i).pos_PA_loc';
end
attach_p=attach_p.*1000; % conversion m -> mm
k=convhull(attach_p);
trisurf(k,attach_p(:,1),attach_p(:,2),attach_p(:,3),'FaceColor',[0.7 0.7 0.7]);
xlabel('[mm]');
ylabel('[mm]');
zlabel('[mm]');

%%% draw HRP in new way
% figure()
% attach_p=zeros(cdpr_parameters.n_cables,3);
% for i = 1:cdpr_parameters.n_cables
%     attach_p(i,:) = cdpr_parameters.cable(i).pos_PA_loc';
% end
% attach_p=attach_p.*1000; % conversion m -> mm
% k=convhull(attach_p);
% % attach_p(end+1,:)=attach_p(1,:);
% % plot3(attach_p(:,1),attach_p(:,2),attach_p(:,3),'LineWidth',1,"Color",'k');
% trisurf(k,attach_p(:,1),attach_p(:,2),attach_p(:,3),'FaceColor',[0.7 0.7 0.7]);
% grid on
% p = gca;
% lim_x = [-300 300];
% lim_y = [-300 300];
% lim_z = [-300 300];
% p.Box = 'Off'; p.LineWidth = 2;
% p.XAxisLocation = 'origin'; p.XLim = lim_x;
% p.XTick = lim_x(1):150:lim_x(2); p.XGrid = 'On';
% p.YAxisLocation = 'origin'; p.ZLim = lim_z;
% p.ZTick = lim_z(1):150:lim_z(2);   p.YGrid = 'On';
% p.YLim = lim_y; p.YTick = lim_y(1):150:lim_y(2);
% p.ZGrid = 'On'; p.GridLineStyle = '-';%'--'
% 
% p.XLabel.String = 'x [m]'; p.XLabel.FontSize = 16;
% p.XLabel.FontWeight = 'bold';
% p.YLabel.String = 'y [m]'; p.YLabel.FontSize = 16;
% p.YLabel.Rotation = 0; p.YLabel.FontWeight = 'bold';
% p.ZLabel.String = 'z [m]'; p.ZLabel.FontSize = 16;
% p.ZLabel.Rotation = 0; p.ZLabel.FontWeight = 'bold';
% 
% p.DataAspectRatioMode = 'manual';
% p.DataAspectRatio= [1;1;1];
% p.CameraPosition = [-400,-500,200];
% title('Piattaforma Finale HRP') % change accordingly to the results

%%% draw ipanema in new way
% figure()
% attach_p=zeros(cdpr_parameters.n_cables,3);
% for i = 1:cdpr_parameters.n_cables
%     attach_p(i,:) = cdpr_parameters.cable(i).pos_PA_loc';
% end
% attach_p=attach_p.*1000; % conversion m -> mm
% k=convhull(attach_p);
% % attach_p(end+1,:)=attach_p(1,:);
% % plot3(attach_p(:,1),attach_p(:,2),attach_p(:,3),'LineWidth',1,"Color",'k');
% trisurf(k,attach_p(:,1),attach_p(:,2),attach_p(:,3),'FaceColor',[0.7 0.7 0.7]);
% grid on
% p = gca;
% lim_x = [-300 300];
% lim_y = [-1200 1200];
% lim_z = [-400 400];
% p.Box = 'Off'; p.LineWidth = 2;
% p.XAxisLocation = 'origin'; p.XLim = lim_x;
% p.XTick = lim_x(1):150:lim_x(2); p.XGrid = 'On';
% p.YAxisLocation = 'origin'; p.ZLim = lim_z;
% p.ZTick = lim_z(1):200:lim_z(2);   p.YGrid = 'On';
% p.YLim = lim_y; p.YTick = lim_y(1):200:lim_y(2);
% p.ZGrid = 'On'; p.GridLineStyle = '-';%'--'
% 
% p.XLabel.String = 'x [m]'; p.XLabel.FontSize = 16;
% p.XLabel.FontWeight = 'bold';
% p.YLabel.String = 'y [m]'; p.YLabel.FontSize = 16;
% p.YLabel.Rotation = 0; p.YLabel.FontWeight = 'bold';
% p.ZLabel.String = 'z [m]'; p.ZLabel.FontSize = 16;
% p.ZLabel.Rotation = 0; p.ZLabel.FontWeight = 'bold';
% 
% p.DataAspectRatioMode = 'manual';
% p.DataAspectRatio= [1;1;1];
% p.CameraPosition = [-500,-1200,200];
% title('Piattaforma Finale IPAnema') % change accordingly to the results

end