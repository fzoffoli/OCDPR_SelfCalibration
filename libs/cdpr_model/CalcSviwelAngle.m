function angle = CalcSviwelAngle(vers_i_loc,vers_j_loc,pos_AD_glob,rot_mat)
%CALCSWIVELANGLE computes the rotation angle of the swivel pulley.
%
%   CALCSWIVELANGLE computes the angle of rotation of the swivel pulley
%   about the fixed z axis, tangent to the pulley at the proximal anchor
%   point D. The following method is based on the definition of an 
%   additional fixed right handed coordinate system, D-xyz attached to D.
%
%   VERS_I is a vector (size[3,1]), containing the components of the unit
%   vector along the x axis, projected on the global frame.
%   VERS_J is a vector (size[3,1]), containing the components of the unit
%   vector along the y axis, projected on the global frame.
%   POS_DA_GLOB is a vector (size[3,1], [m]), containing the components of
%   the position vector (A-D), projected on the global frame.
%
%   ANGLE is the angle of rotation of the swivel pulley [rad].

angle = atan2(dot(rot_mat*vers_j_loc,pos_AD_glob),dot(rot_mat*vers_i_loc,pos_AD_glob));

end