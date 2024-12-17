function [geometric,analitic] = CalcTangencyJacobianCol(vers_n,pos_PD_glob,H_mat,sw_r,vers_u,rot_mat)

  geometric = [-rot_mat*vers_n; -Anti(pos_PD_glob+sw_r*rot_mat*vers_u)*rot_mat*vers_n];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1);
  
end