function [u,up,upp] = Trapezoidal_law(t,v,alpha)

T = 1/(v*(1-alpha));
a = v^2*(1-alpha)/alpha;

if (t>=0 && t<alpha*T)
   %u = 0.5*a*t^2;
   u = (1-alpha)/(2*alpha)*(v*t)^2;
   up = a*t;
   upp = a;
elseif(t>=alpha*T && t<(1-alpha)*T) 
   %u = 0.5*a*alpha^2*T^2+v*(t-alpha*T);
   u = -alpha/(2*(1-alpha))+v*t;
   up = v;
   upp = 0;
else
   %u = 0.5*a*alpha^2*T^2+v*(1-2*alpha)*T+v*(t-(1-alpha)*T)-0.5*a*(t-(1-alpha)*T)^2;
   u = -(2*alpha^2-2*alpha+1)/(2*alpha*(1-alpha))+v*t/alpha-(1-alpha)/(2*alpha)*(v*t)^2;
   up = v-a*(t-(1-alpha)*T);
   upp = -a; 
end

end