%INPUT external workspace limits and vector in unitary space
%OUTPUT vector in cartesian space
function Q = UnitaryToCartesian(H,ext_limits)
    Q = [H(1)*(ext_limits(1,2)-ext_limits(1,1)) + ext_limits(1,1);
        H(2)*(ext_limits(2,2)-ext_limits(2,1)) + ext_limits(2,1);
        H(3)*(ext_limits(3,2)-ext_limits(3,1)) + ext_limits(3,1)];
end