% INPUT geometric and inertial parameters of the robot, index of cable
% couple force controlled
% OUTPUT High Performance Fault Tolerant Workspace volume
function cdpr_outputs = CalcVolHPFT(X,cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,folder,record,ws_info)

%%% Selection of force controlled cables
cableCoupleIndex=X;
cableComb=nchoosek(1:cdpr_parameters.n_cables,cdpr_parameters.n_cables-cdpr_parameters.pose_dim);
cablesForceControlled=cableComb(cableCoupleIndex,:);

% %% DISCRETE WS EVALUATION

cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
    utilities,cdpr_outputs,folder,record,ws_info,cablesForceControlled);

if cdpr_outputs.counter~=0
    %%% UM conversions
    cdpr_outputs.rotCardouSens=180/pi*(cdpr_outputs.rotCardouSens)/1000;

    %%% Accuracy set limits
    for i = 1:cdpr_outputs.counter
        sigma_threshold=[5 0.5];      % mm/mm and deg/mm
        if (cdpr_outputs.posCardouSens(i)<sigma_threshold(1) && cdpr_outputs.rotCardouSens(i)<sigma_threshold(2))
            cdpr_outputs.kaw(i)=1;
        else
            cdpr_outputs.kaw(i)=0;
        end
    end

    % %%% Compute HPFT volume
%     V = nnz(cdpr_outputs.kaw.*cdpr_outputs.teiw)/cdpr_outputs.full_counter; % better use a percentage

    vol_inst = CalcVolInstall(cdpr_parameters,ws_info);
    cdpr_outputs.BoundaryMatrix=diag(cdpr_outputs.kaw)*diag(cdpr_outputs.teiw)*cdpr_outputs.pose(1:3,:)';
    cdpr_outputs.BoundaryMatrix(~any(cdpr_outputs.BoundaryMatrix,2),:)=[];
    [cdpr_outputs.k,vol]=boundary(cdpr_outputs.BoundaryMatrix,0.9);
    cdpr_outputs.V=vol/vol_inst;
else
    cdpr_outputs.V = 0;
end
% figure()
% trisurf(cdpr_outputs.k,cdpr_outputs.BoundaryMatrix(:,1),cdpr_outputs.BoundaryMatrix(:,2),cdpr_outputs.BoundaryMatrix(:,3))
%% BOUNDARY EVALUATION POTT
% InitialPoint=[0; 0; 0];
% for i = 1:cdpr_parameters.n_cables
%     InitialPoint=InitialPoint+cdpr_parameters.cable(i).pos_OD_glob;
% end
% InitialPoint=InitialPoint'/cdpr_parameters.n_cables;
% cdpr_outputs = CalcPottHPFTBoundary(InitialPoint,7,cdpr_parameters,cdpr_variables,ws_info,cablesForceControlled,cdpr_outputs);
% [k,vol]=boundary(cdpr_outputs.BoundaryMatrix,0.9);
% 
% % installation volume
% vol_inst = CalcVolInstall(cdpr_parameters,ws_info);
% 
% cdpr_outputs.k=k;
% cdpr_outputs.V=vol/vol_inst;

%% BOUNDARY EVALUATION ZACCARIA
% cdpr_outputs = CalcWorkspaceBoundaryFlooding(cdpr_parameters,ws_info,cdpr_variables,utilities,cdpr_outputs,cablesForceControlled,record);
% 
% vol_inst = CalcVolInstall(cdpr_parameters,ws_info);
% cdpr_outputs.V=cdpr_outputs.vol/vol_inst;
end