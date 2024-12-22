% INPUT cdpr pose and geometric params
% OUTPUT flag for cable to cable and cable to platform interference
function interf_detected = CableInterfPerreault(cdpr_v,cdpr_p)

% check cc interf for all combinations of cables
cableComb = nchoosek(1:cdpr_p.n_cables,cdpr_p.n_cables-cdpr_p.pose_dim);
interf_cc = 0;
for cIdx=1:length(cableComb)
    cabCouple = cableComb(cIdx,:);

    eps = 1.0e-1;  % numeric zero for planes coincidence
    c_ij = Anti(cdpr_v.cable(cabCouple(1)).pos_BA_glob)*(cdpr_v.cable(cabCouple(2)).pos_BA_glob);
    % a_ij = cdpr_v.cable(cabCouple(2)).pos_OA_glob-cdpr_v.cable(cabCouple(1)).pos_OA_glob;
    b_ij = (cdpr_p.cable(cabCouple(2)).pos_OA_glob-cdpr_v.cable(cabCouple(2)).pos_BA_glob)...
        -(cdpr_p.cable(cabCouple(1)).pos_OA_glob-cdpr_v.cable(cabCouple(1)).pos_BA_glob);
    if abs(c_ij'*b_ij)<eps
        C_ij = [cdpr_v.cable(cabCouple(1)).pos_BA_glob -cdpr_v.cable(cabCouple(2)).pos_BA_glob];
        d_ij = pinv(C_ij)*b_ij;
        if all(d_ij>=0) && all(d_ij<=1)
            interf_cc=1;
        end
    end
end

%%% Cable to platform interference
P=zeros(cdpr_p.n_cables,3);
for j=1:cdpr_p.n_cables
    % P(j,:)=cdpr_v.cable(j).pos_PA_glob';
    P(j,:)=(cdpr_p.cable(j).pos_OA_glob-cdpr_v.cable(j).pos_BA_glob)';
end
vertex_indices=convhull(P);
V(1:length(vertex_indices),:)=vertex_indices(:,1:2);
V(length(vertex_indices)+1:length(vertex_indices)*2,:)=vertex_indices(:,2:3);
V(2*length(vertex_indices)+1:length(vertex_indices)*3,:)=[vertex_indices(:,1) vertex_indices(:,3)];
V_sorted = sort(V,2);
V_shrink=unique(V_sorted,'rows');

%%%--uncomment for graphic representation of the platform edges----%%%
%     figure()
%     for i=1:length(V_shrink)
%         A=P(V_shrink(i,1),:);
%         B=P(V_shrink(i,2),:);
%         plot3([B(1) A(1)],[B(2) A(2)],[B(3) A(3)]);
%         hold on
%         grid on
%     end
%%%----------------------------------------------------------------%%%
interf_cp=0;
eps_cp = 1.0e-3;
for i=1:cdpr_p.n_cables
    for j=1:length(V_shrink)
        e_j=P(V_shrink(j,2),:)'-P(V_shrink(j,1),:)';
        r_ij_s=P(V_shrink(j,1),:)'-(cdpr_p.cable(i).pos_OA_glob-cdpr_v.cable(i).pos_BA_glob);
        % r_ij_t=P(V_shrink(j,2),:)'-(cdpr_v.cable(i).pos_OA_glob-cdpr_v.cable(i).pos_BA_glob);
        v_ij=Anti(cdpr_v.cable(i).pos_BA_glob)*e_j;
        if abs(v_ij'*r_ij_s)<eps_cp
            CE=[cdpr_v.cable(i).pos_BA_glob -e_j];
            d_ij=pinv(CE)*r_ij_s;
            if (d_ij(1)>1.0e-10) && (d_ij(2)>=0) && all(d_ij<=1)
                interf_cp=1;
                break
            end
        end
    end
end


if interf_cp || interf_cc
    interf_detected=1;
else
    interf_detected=0;
end
end