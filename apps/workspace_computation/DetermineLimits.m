function lim = DetermineLimits(cdpr_p,z_lim_inf)

% set initial limits with z_inf parameter
% limits(:,1) = [-Inf;-Inf;z_lim_inf];
% limits(:,2) = [Inf;Inf;Inf];

% determine frame limits as maximum abs value (for const z_inf)
% for i=1:cdpr_p.n_cables
%     point = cdpr_p.cable(i).pos_OD_glob;
%     if i==1
%         limits(:,2) = point;
%         limits(1:2,1) = point(1:2,1);
%     else
%         for j=1:3
%             if (point(j)>limits(j,2))
%                 limits(j,2) = point(j);
%             elseif (point(j)<limits(j,1))
%                 limits(j,1) = point(j);
%             end
%         end
%     end
% end

% determine frame limits as maximum abs value (suggested)
frame_attach_p=zeros(cdpr_p.n_cables,3);
for i=1:cdpr_p.n_cables
    frame_attach_p(i,:)=cdpr_p.cable(i).pos_OD_glob';
end
limits=[min(frame_attach_p)' max(frame_attach_p)'];

% determine platform limits as maximum abs value
platform_attach_p=zeros(cdpr_p.n_cables,3);
for i=1:cdpr_p.n_cables
    platform_attach_p(i,:)=cdpr_p.cable(i).pos_PA_loc';
end
limits2=[min(platform_attach_p)' max(platform_attach_p)'];

% limits symmetric wrt mean computation
lim.xyz_mean = (limits(1:3,1)+limits(1:3,2))./2;
lim.dl_frame = (limits(:,2)-limits(:,1))/2;
lim.dl_plat = abs(limits2(:,2)-limits2(:,1))/2;

end