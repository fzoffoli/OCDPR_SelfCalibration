function [geometric,analitic] = CalcPlatformJacobianCol(vers_t,pos_PD_glob,H_mat,sw_r,vers_u,vers_n,rot_mat)

  geometric = [-rot_mat*vers_t; -Anti(pos_PD_glob+sw_r*rot_mat*(vers_u+vers_n))*rot_mat*vers_t];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1);
  
end