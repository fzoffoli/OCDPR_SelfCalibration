function [geometric,analitic] = CalcTangencyJacobianCol(length,vers_n,pos_PD_glob,H_mat,sw_r,vers_u,vers_w,rot_mat)

  geometric = (1/length)*[-rot_mat*vers_n; -Anti(pos_PD_glob+sw_r*rot_mat*vers_u)*rot_mat*vers_n+length*rot_mat*vers_w];
  geometric = (1/length)*[-rot_mat*vers_n; -Anti(pos_PD_glob+sw_r*rot_mat*vers_u)*rot_mat*vers_n+length*rot_mat*vers_w];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1);
  
end