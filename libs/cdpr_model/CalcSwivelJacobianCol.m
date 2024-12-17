function [geometric,analitic] = CalcSwivelJacobianCol(vers_w,pos_PD_glob,H_mat,pos_DA_glob,rot_mat)

  geometric = [-rot_mat*vers_w; -Anti(pos_PD_glob+pos_DA_glob)*rot_mat*vers_w];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1);
  
end