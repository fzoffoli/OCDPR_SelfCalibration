function angle = CalcTangentAngle(vers_k_loc,sw_r,vers_u,pos_DA_glob, rot_mat)
%CALCTANGENTANGLE computes the supplementary of the wrapped angle of the cable inside the pulley groove.
%
%   VERS_K is a vector (size[3,1]), containing the components of the unit
%   vector along the swivel axis z, projected on the global frame.
%   SW_R is the pulley radius [m].
%   VERS_U is a vector (size[3,1]), containing the components, projected 
%   on the global frame, of the unit vector directed from D to C. 
%   POS_DA_GLOB is a vector(size[3,1],[m]), containing the components of 
%   the position vector (A-D), projected on the global frame.
%
%   ANGLE is the supplementary of the wrapped angle.  


app_v = dot(rot_mat*vers_k_loc,pos_DA_glob)/dot(rot_mat*vers_u,pos_DA_glob);
angle = 2*atan(app_v+sqrt(1 - 2*sw_r/dot(rot_mat*vers_u,pos_DA_glob) + app_v*app_v));
% angle = 2*atan2(app_v+sqrt(1 - 2*sw_r/dot(rot_mat*vers_u,pos_DA_glob) + app_v*app_v),1);

end