function [Z,k] = GenerateConfigPosesBrutal(grid_axis,pose_bounds)

x = linspace(pose_bounds(1,1),pose_bounds(1,2),grid_axis(1));
y = linspace(pose_bounds(2,1),pose_bounds(2,2),grid_axis(2));
z = linspace(pose_bounds(3,1),pose_bounds(3,2),grid_axis(3));

cnt = 1;
for i=1:grid_axis(1)
    for j=1:grid_axis(2)
        for k=1:grid_axis(3)
            Z(:,cnt) = [x(i);y(j);z(k);zeros(3,1)];
            cnt = cnt+1;
        end
    end
end
k = length(Z);
Z = reshape(Z,[k*6 1]);
end