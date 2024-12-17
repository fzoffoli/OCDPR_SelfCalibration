function out=linelaw(t,T)

if t<= 0
    out=0;
elseif t>= T
    out=1;
else
    out=t/T;
end