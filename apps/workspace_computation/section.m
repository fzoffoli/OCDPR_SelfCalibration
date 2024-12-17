%@brief plot sections in dots and in level curves of scatter3 graphs
%
%@params are the pose, index sort order, index, slice direction, number of
%the plane to extract and name of the index
%@returns two figures

function section(x,y,z,criteria_ind,criteria,slice_dir,n_plane, name)
x_=x(criteria_ind);
y_=y(criteria_ind);
z_=z(criteria_ind);
criteria_=criteria(criteria_ind);
eps = 0.001;

switch slice_dir

    case 'x'
        %%%--------x slice-------%%%
        x_grid=unique(x_);
        slice_limits = [x_grid(n_plane)-eps,x_grid(n_plane)+eps];
        x__=x_((x_<slice_limits(2))&(x_>slice_limits(1)));
        y__=y_((x_<slice_limits(2))&(x_>slice_limits(1)));
        z__=z_((x_<slice_limits(2))&(x_>slice_limits(1)));
        criteria__=criteria_((x_<slice_limits(2))&(x_>slice_limits(1)));

        figure();
        cmap = jet(length(criteria__));
        colormap(cmap)
        clim([min(criteria__) max(criteria__)])
        scatter(y__,z__,15,criteria__,'filled');
        colorbar;
        xlabel('Y [m]');
        ylabel('Z [m]');
        title(strcat(name,', x=',string(x_grid(n_plane))));

        y_grid=unique(y__);
        z_grid=unique(z__);
        C=zeros(length(y_grid),length(z_grid));
        for i=1:length(y_grid)
            for j=1:length(z_grid)
                for k=1:length(y__)
                    if y__(k)==y_grid(i) && z__(k)==z_grid(j)
                        C(i,j)=criteria__(k);
                    end
                end
            end
        end
        C(C==0)=NaN;
        figure()
        contour(y_grid,z_grid,C',50,'LineWidth',1.5);
        colorbar;
        xlabel('Y [m]');
        ylabel('Z [m]');
        title(strcat(name,', x=',string(x_grid(n_plane))));
    case 'y'
        %%%--------y slice-------%%%
        y_grid=unique(y_);
        slice_limits = [y_grid(n_plane)-eps,y_grid(n_plane)+eps];
        x__=x_((y_<slice_limits(2))&(y_>slice_limits(1)));
        y__=y_((y_<slice_limits(2))&(y_>slice_limits(1)));
        z__=z_((y_<slice_limits(2))&(y_>slice_limits(1)));
        criteria__=criteria_((y_<slice_limits(2))&(y_>slice_limits(1)));

        figure();
        cmap = jet(length(criteria__));
        colormap(cmap)
        clim([min(criteria__) max(criteria__)])
        scatter(x__,z__,15,criteria__,'filled');
        colorbar;
        xlabel('X [m]');
        ylabel('Z [m]');
        title(strcat(name,', y=',string(y_grid(1))));

        x_grid=unique(x__);
        z_grid=unique(z__);
        C=zeros(length(x_grid),length(z_grid));
        for i=1:length(x_grid)
            for j=1:length(z_grid)
                for k=1:length(x__)
                    if x__(k)==x_grid(i) && z__(k)==z_grid(j)
                        C(i,j)=criteria__(k);
                    end
                end
            end
        end
        C(C==0)=NaN;
        figure()
        contour(x_grid,z_grid,C',50,'LineWidth',1.5);
        colorbar;
        xlabel('X [m]');
        ylabel('Z [m]');
        title(strcat(name,', y=',string(y_grid(n_plane))));
    case 'z'
        %%%--------z slice-------%%%
        z_grid=unique(z_);
        slice_limits = [z_grid(n_plane)-eps,z_grid(n_plane)+eps];
        x__=x_((z_<slice_limits(2))&(z_>slice_limits(1)));
        y__=y_((z_<slice_limits(2))&(z_>slice_limits(1)));
        z__=z_((z_<slice_limits(2))&(z_>slice_limits(1)));
        criteria__=criteria_((z_<slice_limits(2))&(z_>slice_limits(1)));

        figure();
        cmap = jet(length(criteria__));
        colormap(cmap)
        clim([min(criteria__) max(criteria__)])
        scatter(x__,y__,15,criteria__,'filled');
        colorbar;
        xlabel('X [m]');
        ylabel('Y [m]');
        title(strcat(name,', z=',string(z_grid(n_plane))));

        x_grid=unique(x__);
        y_grid=unique(y__);
        C=zeros(length(x_grid),length(y_grid));
        for i=1:length(x_grid)
            for j=1:length(y_grid)
                for k=1:length(y__)
                    if x__(k)==x_grid(i) && y__(k)==y_grid(j)
                        C(i,j)=criteria__(k);
                    end
                end
            end
        end
        C(C==0)=NaN;
        figure()
        [~,h]=contour(x_grid,y_grid,C',50,'LineWidth',1.5);
        h.ContourZLevel=z_grid(n_plane);
        colorbar;
        view(3)
        grid minor
        xlabel('X [m]');
        ylabel('Y [m]');
        zlabel('Z [m]');
        title(strcat(name,', z=',string(z_grid(n_plane))));
end