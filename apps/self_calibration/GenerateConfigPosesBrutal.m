function [Z,k] = GenerateConfigPosesBrutal(ws_info,pose_bounds)

x = linspace(pose_bounds(1,1),pose_bounds(1,2),ws_info.mesh_divider);
y = linspace(pose_bounds(2,1),pose_bounds(2,2),ws_info.mesh_divider);
z = linspace(pose_bounds(3,1),pose_bounds(3,2),ws_info.mesh_divider);

cnt = 1;
for i=1:ws_info.mesh_divider
    for j=1:ws_info.mesh_divider
        for k=1:ws_info.mesh_divider
            Z(:,cnt) = [x(i);y(j);z(k);zeros(3,1)];
            cnt = cnt+1;
        end
    end
end
k = length(Z);
Z = reshape(Z,[k*6 1]);
end