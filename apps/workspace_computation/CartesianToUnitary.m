%INPUT external workspace limits (installation space) and vector in
%cartesian space
%OUTPUT vector in unitary space.
function H = CartesianToUnitary(Q,ext_limits)
    H = [(Q(1)-ext_limits(1,1))/(ext_limits(1,2)-ext_limits(1,1));
        (Q(2)-ext_limits(2,1))/(ext_limits(2,2)-ext_limits(2,1));
        (Q(3)-ext_limits(3,1))/(ext_limits(3,2)-ext_limits(3,1))];
end