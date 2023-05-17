function Xdot = trapezoidalP2P(t,X,x,y,theta)

tf = 1; tb = 1/3;

V = @(P1,P2) (P2-P1)*1.5;%maximum velocity 

if t <= tb && t >= 0
    pdotFun = @(P1,P2) V(P1,P2)/tb*t;
elseif t > tb && t <= tf - tb
    pdotFun = @(P1,P2) V(P1,P2);
elseif t > tf - tb && t <= tf
    pdotFun = @(P1,P2) (V(P1,P2)/tb)*(1 - t);
end

w = pdotFun(theta(1),theta(2));
v = sqrt(pdotFun(x(1),x(2))^2 + pdotFun(y(1),y(2))^2);

Xdot = [v*cos(X(3)) ; v*sin(X(3)) ; w];

end
