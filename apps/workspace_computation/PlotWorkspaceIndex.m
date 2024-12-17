function f = PlotWorkspaceIndex(cdpr_p,cdpr_v,out,ws_info,folder,simulation_title,config_name,cablesForceControlled)

%%% This part of code was added to divide the computation by the plot/save
rec = RecordType(cdpr_p,simulation_title,config_name,cablesForceControlled);
if cdpr_p.n_cables ~= 4
    try
        wp_name = strcat(config_name,'_WS.mat');
        load(wp_name);
        rec = rec.ResetFigureLimits(ws_data.limits,8);
    catch
        ws_data = [];
    end
    % rec.SetFrame(cdpr_v,cdpr_p);
    rec = rec.ResetFigureLimits(out.limits,ws_info.display_grid_divider);%%% be careful on erasing data
end

cableIdx=string(cablesForceControlled);
if cdpr_p.pose_dim==3
    filename = strcat(folder,'/workspace_files/',rec.figure_handle.FileName,cableIdx(1));
else
    filename = strcat(folder,'/workspace_files/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end));
end
switch ws_info.display_criteria
    case DisplayCriteria.NONE
        name = strcat(filename,'_no_criteria');
    case DisplayCriteria.POSITION_SENSITIVITY
        for i=1:out.counter
            cdpr_v = UpdateIKZeroOrd(out.position(:,i),out.ang_par(:,i),cdpr_p,cdpr_v);
            Kr = cdpr_v.geometric_jacobian(4:6,:)';
            Kp = cdpr_v.geometric_jacobian(1:3,:)';
            pr =  Kr*MyInv(Kr'*Kr)*Kr';
            Pr = eye(length(pr))-pr;
            out.manipP(1,i) = sqrt(norm(MyInv(Kp'*Pr*Kp),2));
        end
        criteria = out.manipP;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_pos_sens');
    case DisplayCriteria.ORIENTATION_SENSITIVITY
        for i=1:out.counter
            cdpr_v = UpdateIKZeroOrd(out.position(:,i),out.ang_par(:,i),cdpr_p,cdpr_v);
            Kr = cdpr_v.geometric_jacobian(4:6,:)';
            Kp = cdpr_v.geometric_jacobian(1:3,:)';
            pp = Kp*MyInv(Kp'*Kp)*Kp';
            Pp = eye(length(pp))-pp;
            out.manipR(1,i) = sqrt(norm(MyInv(Kr'*Pp*Kr),2));
        end
        criteria = out.manipR;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_orient_sens');
    case DisplayCriteria.TENSION_SENSITIVITY
        criteria = out.WS_perf;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_sens');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_tension_sens');
        figtitle = strcat('TensionSens index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.MANIPULABILITY
    case DisplayCriteria.MAX_TENSION
        criteria = max(out.tension_vector);
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_max_tension');
    case DisplayCriteria.MIN_TENSION
        criteria = min(out.tension_vector);
        [~,criteria_ind] = sort(criteria);
        name  = strcat(filename,'_min_tension');
    case DisplayCriteria.TENSION_SENSITIVITY_2%%%%%%%%%
        criteria = out.sigma;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_sens2');
    case DisplayCriteria.TENSION_SENSITIVITYmap%%%%%%%%%
        criteria = out.index;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_sensmap');
    case DisplayCriteria.TENSION_SENSITIVITYHeurmap%%%%%%%%%new
        criteria = out.heurindex;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_sensHeurmap');
    case DisplayCriteria.TENSION_SENSITIVITYpair%%%%%%%%%new
        criteria = out.sensitivity;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_senspair');
    case DisplayCriteria.TENSION_SENSITIVITYcontour%%%%%%%%%
        criteria = out.index;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_senscont');
    case DisplayCriteria.TENSION_SENSITIVITYsvdmin%%%%%%%%%
        criteria = out.index;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_senscont');
    case DisplayCriteria.TENSION_SENSITIVITYmultiplicity%%%%%%%%%
        criteria = out.mul;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_senscont');
    case DisplayCriteria.CABLE_SENSITIVITYrot
        criteria = out.rotSensitivity;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_cable_sensRot');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_cable_sensRot');
        figtitle = strcat('RotSens index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.CABLE_SENSITIVITYpos
        criteria = out.posSensitivity;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_cable_sensPos');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_cable_sensPos');
        figtitle = strcat('PosSens index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.CABLE_SENSITIVITYrotCardou
        criteria = out.rotCardouSens;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_cable_sensRotCard');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_cable_sensRotCard');
        figtitle = strcat('RotSens index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.CABLE_SENSITIVITYposCardou
        criteria = out.posCardouSens;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_cable_sensPosCard');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_cable_sensPosCard');
        figtitle = strcat('PosSens index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.TENSION_SENSITIVITYinputRatio
        criteria = out.sigmaTauL;
        [~,criteria_ind] = sort(criteria);
        name = strcat(filename,'_tension_inputRatio');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_tension_inputRatio');
        figtitle = strcat('Input ratio index, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.TENSION_ERR_INSENSITIVE_WS
        criteria = out.teiw;
        criteria_ind = find(criteria);
        not_criteria_ind = find(~criteria);
        name = strcat(filename,'_teiw');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_teiw');
        figtitle = strcat('Tension Error Insensitive WS, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.KIN_ACCURATE_WS
        criteria = out.kaw;
        criteria_ind = find(criteria);
        name = strcat(filename,'_teiw');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_kaw');
        figtitle = strcat('Kinematic Accurate WS, cables ', cableIdx(1),cableIdx(end));
    case DisplayCriteria.KA_TEI_WS
        name = strcat(filename,'_ka_tei_ws');
        pdfname = strcat(folder,'/workspace_files/PDF_figures/',rec.figure_handle.FileName,cableIdx(1),cableIdx(end),'_kaw');
        figtitle = strcat('Kinematic Accurate and Tension error-insensitive WS, cables ', cableIdx(1),cableIdx(end));
    otherwise
        disp('No workspace display method selected')
        flag_vis = 0;

end

switch ws_info.workspace_type
    case WorkspaceTypes.TRANSLATIONAL
%         name  = strcat(name,',eps = [',num2str(out.pose(4,1)),';',num2str(out.pose(5,1)),';',num2str(out.pose(6,1)),']m');
        x = out.pose(1,:);
        y = out.pose(2,:);
        z = out.pose(3,:);
        if (ws_info.display_criteria == DisplayCriteria.NONE)
            f = scatter3(x,y,z,20,'k','filled');
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYmap)%%%%%%%%%%%%
            cmap=createSensMap(cdpr_p,cdpr_v,criteria,ws_info);
            % colorbar;
            com=nchoosek(length(cdpr_v.cable),(cdpr_p.n_cables-cdpr_p.pose_dim));
            set(colorbar,'Ticks',1:1:com);
            colormap(cmap)
            caxis([min(criteria) max(criteria)])
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),30,cmap,'filled');  %cmap,

        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYHeurmap)%%%%%%%%%%%%
            cmap=createSensMap(cdpr_p,cdpr_v,criteria,ws_info);
            % colorbar;
            com=nchoosek(length(cdpr_v.cable),(cdpr_p.n_cables-cdpr_p.pose_dim));
            set(colorbar,'Ticks',1:1:com);
            colormap(cmap)
            caxis([min(criteria) max(criteria)])
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),30,cmap,'filled');  %cmap,
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYpair)%%%%%%%%%%%%
            [xv,yv,zv] = meshgrid(linspace(min(x), max(x), numel(x)/10),linspace(min(y), max(y), numel(y)/10),linspace(min(z), max(z), numel(z)/10));
            for kpair=1:out.combi(:,:,1)

                %
                MFm = griddata(x,y,z,out.sensitivity(kpair,:), xv, yv,zv);
                [ai,aj]=find(out.coM==kpair);
                if (cdpr_p.n_cables-cdpr_p.pose_dim==1)
                    zlev = -0.4:0.02:0.7;%falco
                    ylev = -0.85:0.02:0.85;
                    xlev = -0.85:0.02:0.85;
                    %               set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
                    %               set(colorbar,'Ticks',[0  0.1 0.2  0.3 0.4 0.5 0.6 0.7 0.8 0.9 1]);
                else
                    %uncomment for HRP
                    %
                    zlev = 0.1:0.01:2.8;
                    ylev = -1.5:0.1:1.5;
                    xlev = -3.6:0.1:3.6;

                    %set(gca,'clim',[0 uplim]) %set(gca,'ColorScale','log','clim',[1 2])
                    %set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
                    %
                    % uncomment for ipa:
                    %
                    %                   zlev = -2.5:0.01:2.5;
                    %                   ylev = -5.5:0.1:5.5;
                    %                   xlev = -8.5:0.1:8.5;
                    %                   set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
                    %                   set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
                    %
                end
                uplim=max(out.sensitivity(kpair,:));
                fprintf('max sens value for pair   %d - %d = %f N',ai,aj, uplim)
                fprintf('\n')
                U=0;D=U;A=D;
                for ip=1:length(out.sensitivity(kpair,:))
                    if out.sensitivity(kpair,ip)<=2
                        U=U+1;
                    elseif out.sensitivity(kpair,ip)>2 && out.sensitivity(kpair,ip)<=10
                        D=D+1;
                    else
                        A=A+1;
                    end
                end
                XX=[U D A];
                figure()
                labels = {'\sigma \leq 2','2 < \sigma \leq 10','\sigma \geq 10'};
                %                 pie(XX,labels)
                pie(XX)
                title(sprintf('sensitivity for force-controlled cable pair %d - %d',ai,aj))
                lgd = legend(labels);
                %           figure%
                saveas(gcf,[path,'\pie_',num2str(ai),'-',num2str(aj),'.fig'])



                %open new figure
                [cdpr_parameters, cdpr_variables, culo ,cdpr_outputs,rec,utilities] = ...
                    LoadConfigAndInit("8_cable_HRPCable","8_cable_HRPCable");

                grid on
                contourslice(xv,yv,zv,MFm,[],[],zlev)
                hold on
                contourslice(xv,yv,zv,MFm,[],ylev,[])
                hold on
                contourslice(xv,yv,zv,MFm,xlev,[],[])
                hold on
                title(sprintf('sensitivity for force-controlled cable pair %d - %d',ai,aj))
                colormap('jet')
                colorbar

                saveas(gcf,[path,'\contour_',num2str(ai),'-',num2str(aj),'.fig'])


                %                   figure()
                %                   plot(out.sensitivity(kpair,:))
            end
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYcontour)%%%%%%%%%%%%
            [xv,yv,zv] = meshgrid(linspace(min(x), max(x), numel(x)/10),linspace(min(y), max(y), numel(y)/10),linspace(min(z), max(z), numel(z)/10));
            MFm = griddata(x,y,z,out.sigma, xv, yv,zv);
            colormap('jet')
            title(sprintf('min FD sensitivity'))
            colorbar
            if (cdpr_p.n_cables-cdpr_p.pose_dim==1)
                zlev = -0.4:0.02:0.7;%falcon
                ylev = -0.85:0.02:0.85;
                xlev = -0.85:0.02:0.85;
                %               set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
                %               set(colorbar,'Ticks',[0  0.1 0.2  0.3 0.4 0.5 0.6 0.7 0.8 0.9 1]);
            else
                zlev = -2.5:0.01:2.5; %ipa
                ylev = -5.5:0.1:5.5;
                xlev = -8.5:0.1:8.5;
                set(gca,'clim',[1 2]) %set(gca,'ColorScale','log','clim',[1 2])
                set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            end
            grid on
            contourslice(xv,yv,zv,MFm,[],[],zlev)
            hold on
            contourslice(xv,yv,zv,MFm,[],ylev,[])
            hold on
            contourslice(xv,yv,zv,MFm,xlev,[],[])
            hold on


            %           figure
            %           colormap('jet')
            %           title(sprintf('min FD sensitivity for x=-0.3'))
            %           colorbar
            %          % set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
            %           %set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            %           grid on
            %           contourslice(xv,yv,zv,MFm,-0.3,[],[])
            %           view(3)
            %           axis equal
            % axis([-0.6 0.3 -0.8 0.2 0.2 1.2])
            %
            %           figure
            %           colormap('jet')
            %           title(sprintf('min FD sensitivity for y=0.3'))
            %           colorbar
            % %           set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
            % %           set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            %           grid on
            %           contourslice(xv,yv,zv,MFm,[],0.3,[])
            %           view(3)
            %           axis equal
            % axis([-0.3 0.3 -0.6 0.6 0.2 1.2])
            %
            %           figure
            %           colormap('jet')
            %           title(sprintf('min FD sensitivity for z=0.8'))
            %           colorbar
            % %           set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
            % %           set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            %           grid on
            %           contourslice(xv,yv,zv,MFm,[],[], 0.8)
            %           view(3)
            %           axis equal
            % axis([-0.7 0.7 -0.7 0.7 -0.4 1.5])

            %ipa
            figure
            colormap('jet')
            title(sprintf('min FD sensitivity for x=0'))
            colorbar
            set(gca,'clim',[1 2]) %set(gca,'ColorScale','log','clim',[1 2])
            set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            grid on
            contourslice(xv,yv,zv,MFm,0,[],[])
            view(3)
            axis([-8 8 -5 5 -3 3])
            axis equal
            figure
            colormap('jet')
            title(sprintf('min FD sensitivity for y=0'))
            colorbar
            set(gca,'clim',[1 2]) %set(gca,'ColorScale','log','clim',[1 2])
            set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            grid on
            contourslice(xv,yv,zv,MFm,[],0,[])
            view(3)
            axis([-8 8 -5 5 -3 3])
            axis equal
            figure
            colormap('jet')
            title(sprintf('min FD sensitivity for z=1.6'))
            colorbar
            set(gca,'clim',[1 2]) %set(gca,'ColorScale','log','clim',[1 2])
            set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            grid on
            contourslice(xv,yv,zv,MFm,[],[],1.6)
            view(3)
            axis([-8 8 -5 5 -3 3])
            axis equal

            %svd
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYsvdmin)%%%%%%%%%%%%
            [xv,yv,zv] = meshgrid(linspace(min(x), max(x), numel(x)/10),linspace(min(y), max(y), numel(y)/10),linspace(min(z), max(z), numel(z)/10));
            MFm = griddata(x,y,z,out.svd, xv, yv,zv);
            colormap('jet')
            title(sprintf('min FD sensitivity'))
            colorbar
            if (cdpr_p.n_cables-cdpr_p.pose_dim==1)
                zlev = -0.4:0.02:0.7;%falcon
                ylev = -0.85:0.02:0.85;
                xlev = -0.85:0.02:0.85;
                %               set(gca,'clim',[0 2]) %set(gca,'ColorScale','log','clim',[1 2])
                %               set(colorbar,'Ticks',[0  0.1 0.2  0.3 0.4 0.5 0.6 0.7 0.8 0.9 1]);
            else
                zlev = -2.5:0.01:2.5; %ipa
                ylev = -5.5:0.1:5.5;
                xlev = -8.5:0.1:8.5;
                %set(gca,'clim',[1 2]) %set(gca,'ColorScale','log','clim',[1 2])
                %set(colorbar,'Ticks',[1  1.1 1.2  1.3 1.4 1.5 1.6 1.7 1.8 1.9 2]);
            end
            grid on
            contourslice(xv,yv,zv,MFm,[],[],zlev)
            hold on
            contourslice(xv,yv,zv,MFm,[],ylev,[])
            hold on
            contourslice(xv,yv,zv,MFm,xlev,[],[])
            hold on
            %

        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYmultiplicity)%%%%%%%%%%%%
            cmap1=createSensMap(cdpr_p,cdpr_v,criteria, ws_info);
            %           cmap = jet(length(criteria));
            colorbar;
            colormap(cmap1)
            caxis([min(criteria) max(criteria)])
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap1,'*');
            f.SizeData = 50;
        elseif (ws_info.display_criteria == DisplayCriteria.CABLE_SENSITIVITYrot) %%% This section was added by FZ to show the rotational kinematic index
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
            title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.CABLE_SENSITIVITYpos) %%% This section was added by FZ to show the positional kinematic index
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
            title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.CABLE_SENSITIVITYrotCardou) %%% This section was added by FZ to show the rotational cardou kinematic index
            [x,y,z,criteria,criteria_ind]=TeiwOutManipulation(x,y,z,criteria,out);   %%% uncomment for teiw evaluation
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
            title(figtitle);
            %%%-section
            section(x,y,z,criteria_ind,criteria,'x',2,figtitle);
            %%%-------------
        elseif (ws_info.display_criteria == DisplayCriteria.CABLE_SENSITIVITYposCardou) %%% This section was added by FZ to show the rotational cardou kinematic index
            [x,y,z,criteria,criteria_ind]=TeiwOutManipulation(x,y,z,criteria,out);   %%% uncomment for teiw evaluation
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
            title(figtitle);
            %%%-section
            section(x,y,z,criteria_ind,criteria,'x',2,figtitle);
            %%%-------------
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYinputRatio) %%% This section was added by FZ to show the input ratio index
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
            title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITY)
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            clim([min(criteria) max(criteria)])
%             f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,criteria(criteria_ind),'filled');
%             title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.TENSION_ERR_INSENSITIVE_WS)
%             [x,y,z,criteria]=CriteriaOutManipulation(x,y,z,criteria);   %%% uncomment for teiw evaluation
            f = scatter3(x,y,z,10,~criteria,'filled');
            colormap(gca,"parula");
%             title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.KIN_ACCURATE_WS)
            [x,y,z,criteria_out]=CriteriaOutManipulation(x,y,z,criteria,criteria);   %%% uncomment for teiw evaluation
            f = scatter3(x,y,z,10,~criteria_out,'filled');
            colormap(gca,"parula");
%             title(figtitle);
        elseif (ws_info.display_criteria == DisplayCriteria.KA_TEI_WS)
            ka_tei_ws = out.kaw.*out.teiw;
            [x_,y_,z_,teiw_]=CriteriaOutManipulation(out.pose(1,:),out.pose(2,:),out.pose(3,:),~out.teiw,~ka_tei_ws);
            scatter3(x_,y_,z_,10,teiw_,'filled');
            map = [ 1 165/255 0; 0 128/255 128/255];
            colormap(gca,map);
            hold on
            set(gca,'linewidth',1.5)
            [x,y,z,~]=CriteriaOutManipulation(out.pose(1,:),out.pose(2,:),out.pose(3,:),ka_tei_ws,ka_tei_ws);
            f=scatter3(x,y,z,10,[128/255 0 128/255],'filled');
            %             title(figtitle);
            if strcmp(rec.figure_handle.FileName,"8_cable_HRPCable")
%                 title(strcat('HRP cable force controlled','{}',cableIdx(1),'-',cableIdx(2)),'FontSize',16);
            else
%                 title(strcat('Ipanema cable force controlled','{}',cableIdx(1),'-',cableIdx(2)),'FontSize',16);
            end
            figure()
            set(gcf, 'Position',  [250, 250, 600, 300])
            p=pie(out.X_pie);
            p(1).FaceColor = [1 1 1];
            p(2).FontSize = 16;
            p(3).FaceColor = [0 128/255 128/255];
            p(4).FontSize = 16;
            p(5).FaceColor = [1 165/255 0];
            p(6).FontSize = 16;
            p(7).FaceColor = [128/255 0 128/255];
            p(8).FontSize = 16;
            legend('\textit{NFW}','\textit{TS}','\textit{KS}','\textit{WFEI}','FontSize',16,'Location', 'westoutside','Interpreter','latex');
        else
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            %                 caxis([min(criteria) max(criteria)])   %%%NBB this code was commented to let the "TENSION_SENSITIVITY" graphics work
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
        end
    case WorkspaceTypes.ORIENTATION
        close all
        name  = strcat(name,',p = [',num2str(out.pose(1,1)),';',num2str(out.pose(2,1)),';',num2str(out.pose(3,1)),']m');
        pax = polaraxes;
        polaraxes(pax);
        th = out.pose(4,:);
        r = out.pose(5,:)*180/pi;
        cmap = parula(length(criteria));
        polarscatter(th(criteria_ind),r(criteria_ind),10,cmap,'filled')
        colorbar;
        colormap(cmap)
        caxis([min(criteria) max(criteria)])
        pax.FontWeight = 'bold';
        pax.RTick = [20 40 60 80];
        pax.Layer = 'top';
        pax.GridAlpha = 1;
        pax.RAxisLocation = 10;
    case WorkspaceTypes.REACHABLE
        x = out.pose(1,:);
        y = out.pose(2,:);
        z = out.pose(3,:);
        if (ws_info.display_criteria == DisplayCriteria.NONE)
            f = scatter3(x,y,z,20,'k','filled');
        else
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            caxis([min(criteria) max(criteria)])
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),20,cmap,'filled');%10
        end
    case WorkspaceTypes.TOTAL_ORIENTATION
        name  = strcat(name,',eps_i = [',num2str(ws_info.lim(1,1)),';',num2str(ws_info.lim(2,1)),';',num2str(ws_info.lim(3,1)),']rad, '...
            ,',eps_s = [',num2str(ws_info.lim(1,2)),';',num2str(ws_info.lim(2,2)),';',num2str(ws_info.lim(3,2)),']rad');
        x = out.pose(1,:);
        y = out.pose(2,:);
        z = out.pose(3,:);
        if (ws_info.display_criteria == DisplayCriteria.NONE)
            f = scatter3(x,y,z,20,'k','filled');
        else
            cmap = jet(length(criteria));
            colorbar;
            colormap(cmap)
            caxis([min(criteria) max(criteria)])
            f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
        end
end
savefig(rec.figure_handle,strcat(name,'.fig'));

%%% Uncomment to get .pdf format
% print(rec.figure_handle,gcf,'-dpdf',pdfname);
end