function tau_c=ComputeBarycenterDistribution(v)

A=0;
for i = 1:length(v)-1
    A=A+0.5*(v(1,i)*v(2,i+1)-v(1,i+1)*v(2,i));
end
for i = 1:length(v)-1
    tau_c(1)=(v(1,i)+v(1,i+1))*(v(1,i)*v(2,i+1)-v(1,i+1)*v(2,i))/(6*A);
    tau_c(2)=(v(2,i)+v(2,i+1))*(v(1,i)*v(2,i+1)-v(1,i+1)*v(2,i))/(6*A);
end
end