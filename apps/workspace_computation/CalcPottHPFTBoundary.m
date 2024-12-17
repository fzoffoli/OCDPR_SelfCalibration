%%% this function compute the boundary points of the workspace that satisfy
%%% a boolean function g() for a given OCDPR
%%% INPUT cdpr parameters
%%% OUTPUT matrix 3xn of boundary points (is a convex hull)

function out = CalcPottHPFTBoundary(InitialPoint,ScaleFactor,cdpr_p,cdpr_v,ws_info,cableIdx,out)

% installation boundaries
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);
ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat.*1.1 lim.xyz_mean+lim.dl_frame-lim.dl_plat.*1.1]; % /2
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe+0;
UpperBound=ws_limits(:,2)';
LowerBound=ws_limits(:,1)';

%octahedron centered in [0;0;0] with edges length sqrt(2)
V=zeros(6,3);
V(1,:)=[1,0,0];
V(2,:)=[-1,0,0];
V(3,:)=[0,1,0];
V(4,:)=[0,-1,0];
V(5,:)=[0,0,1];
V(6,:)=[0,0,-1];

% V=V*ScaleFactor+InitialPoint;
V=V*RotZ(pi/4);
k=convhull(V);

%vertices generation
n_iter = 3;
vertices = [];
for i=1:8
    main_triangle = V(k(i,:),:);
    main_triangle(4,:) = main_triangle(1,:);
    vertices=recursive_subdivide(main_triangle,n_iter,vertices);
end
vertices=unique(vertices,'rows');

%Line Search
eps = 0.01;
out.BoundaryMatrix=zeros(size(vertices));
out.counter=0;
for i=1:length(vertices)
    lambda_min = 0;
    lambda_max = ScaleFactor;
    while lambda_max-lambda_min>eps
        lambda = 0.5*(lambda_max+lambda_min);
        position = InitialPoint+lambda*vertices(i,:);
        [flag,out]=BoolWsCriterion(position',cdpr_p,cdpr_v,ws_info,cableIdx,out);
        if flag && all(position<=UpperBound) && all(position>=LowerBound)
            lambda_min=lambda;
        else
            lambda_max=lambda;
        end
    end
    out.BoundaryMatrix(i,:)=position;
end
end