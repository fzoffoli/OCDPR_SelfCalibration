function [sigmaTauL, tauP_dl, tauN_dl, dJ_ort]=InputRatioIndex(cdpr_v,cdpr_p,cableIndexes,Jd,Jc,tau_c,delta_l)
%%%NB BE CAREFUL WITH THE PERMUTATION MATRIX FOR THE JACOBIAN BLOCKS

%%%%% This code was copied from the function CalcStiffnessMat.m to compute Jacobian tensor
ii=cableIndexes(1);
jj=cableIndexes(end);
n = cdpr_p.n_cables;
n_d= cdpr_p.pose_dim;
dJ = zeros(n_d,n_d,n);

for i=1:n
    if cdpr_p.pose_dim>3
        T = CalcMatrixT(cdpr_v.cable(i));
        a_tilde = Anti(cdpr_v.cable(i).pos_PA_glob);
        dJ(:,:,i) = [T -T*a_tilde;a_tilde*T (-a_tilde*T+Anti(cdpr_v.cable(i).vers_t))*a_tilde]*cdpr_v.platform.D_mat;
    else    %%% here is computed the jacobian tensor for the planar case
        T = CalcMatrixT_PLN(cdpr_v.cable(i));
        pos_PA_glob=[cdpr_v.cable(i).pos_PA_glob(1); cdpr_v.cable(i).pos_PA_glob(3)];
        a_tilde = Anti_PLN(pos_PA_glob);
        vers_t=[cdpr_v.cable(i).vers_t(1); cdpr_v.cable(i).vers_t(3)];
        db_dZeta=[0 0 pos_PA_glob'*Rot2d(pi)'*vers_t] + pos_PA_glob'*Rot2d(pi/2)'*[T -T*a_tilde'];
        dJ(:,:,i) = [T -T*a_tilde'; db_dZeta];
    end
end

%%%%% Derivative of Jacobian controlled blocks wrt pose
Jd_dz=dJ;
if ii~=jj
    if ii>jj
        Jd_dz(:,:,ii)=[];
        Jd_dz(:,:,jj)=[];
        Jc_dz(:,:,1)=dJ(:,:,jj);
        Jc_dz(:,:,2)=dJ(:,:,ii);
    else
        Jd_dz(:,:,jj)=[];
        Jd_dz(:,:,ii)=[];
        Jc_dz(:,:,1)=dJ(:,:,ii);
        Jc_dz(:,:,2)=dJ(:,:,jj);
    end
else
   Jd_dz(:,:,ii)=[];
   Jc_dz(:,:)=dJ(:,:,ii);
end


%%%%% Derivative of external wrench wrt pose (hip only gravity)
if cdpr_p.pose_dim>3
    E=[zeros(3,6); zeros(3) -Anti(cdpr_v.platform.ext_load(1:3))*Anti(cdpr_v.platform.pos_PG_glob)];
    W_dz=-E*cdpr_v.platform.D_mat;
else

    E=Anti_PLN(cdpr_v.platform.pos_PG_glob)*Rot2d(pi)'*[0; cdpr_v.platform.ext_load(3)];
    W_dz=[zeros(2,3); 0 0 E];
end

%%%%% Computation of d_tau_P/d_l
if n_d>3
    ext_wrench=cdpr_v.platform.ext_load;
else
    ext_wrench=[0; cdpr_v.platform.ext_load(3); 0];
end
tau0_dz_1=zeros(n_d);
tau0=(Jd\eye(n_d))*ext_wrench;
for i=1:size(Jd_dz,3)
    tau0_dz_1=tau0_dz_1+Jd_dz(:,:,i)*tau0(i);
end
tau0_dz=(Jd\eye(n_d))*(-tau0_dz_1+W_dz);
tauP_dz=[tau0_dz; zeros(n-n_d,n_d)];
tauP_dl=tauP_dz*[(Jd'*cdpr_v.D_mat)\eye(n_d) zeros(n_d,n-n_d)];

%%%% Computation of the d_tau_N/d_l
tauN_dz_1=zeros(n_d);
tauN=[-(Jd\eye(n_d))*Jc; eye(n-n_d)]*tau_c;
for j=1:size(dJ,3)
    if j<=size(Jd_dz,3)
        tauN_dz_1=tauN_dz_1+Jd_dz(:,:,j)*tauN(j);
    else
        tauN_dz_1=tauN_dz_1+Jc_dz(:,:,j-n_d)*tauN(j);
    end
end
tauN_dz=[ -(Jd\eye(n_d))*tauN_dz_1; zeros(n-n_d,n_d)];
tauN_dl=tauN_dz*[(Jd'*cdpr_v.D_mat)\eye(n_d) zeros(n_d,n-n_d)];

%%%% Computation of dJ_ort/dZeta
dJ_perm(:,:,1:n_d)=Jd_dz;
dJ_perm(:,:,n_d+1:n)=Jc_dz;
dJ_perm=permute(dJ_perm,[1 3 2]);
J_ort=[-(Jd\eye(n_d))*Jc; eye(n-n_d)];
for k=1:n_d
    T1(:,:,k)=-(Jd\eye(n_d))*dJ_perm(:,:,k)*J_ort;
end
T1=permute(T1,[1 3 2]);
dJ_ort_dz=T1;
dJ_ort_dz(n_d+1:n,:,:)=zeros(n-n_d,n_d,n-n_d);

%%%% Computation of dJ_ort/dl*delta_l
J_par_T=[(Jd'*cdpr_v.D_mat)\eye(n_d) zeros(n_d,n-n_d)];
dJ_ort=zeros(n,n-n_d);
for k=1:n-n_d
    dJ_ort(:,k)=dJ_ort_dz(:,:,k)*J_par_T*delta_l;
end

%%%% Input ratio index as infinity norm of the sensitivity matrix
tau_dl=tauP_dl+tauN_dl;
sigmaTauL=norm(tau_dl,'inf');
end