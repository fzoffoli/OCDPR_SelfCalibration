function matrix=CalcMatrixT_PLN(cable_v)

pos_BA_glob=[cable_v.pos_BA_glob(1); cable_v.pos_BA_glob(3)]; %hio plane y=0
matrix=eye(2)*norm(pos_BA_glob);

end