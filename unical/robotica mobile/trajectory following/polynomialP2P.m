function Xdot = polynomialP2P(t,X,x,y,theta)

syms s a0 a1 a2 a3 a4 a5;
lambdas = a0*s^5 + a1*s^4 + a2*s^3 + a3*s^2 +a4*s + a5;
lambdadots = diff(lambdas); lambdaddots = diff(lambdadots);
%conditions
c1 = subs(lambdas,s,0) == 0; c4 = subs(lambdas,s,1) == 1;
c2 = subs(lambdadots,s,0) == 0; c5 = subs(lambdadots,s,1) == 0;
c3 = subs(lambdaddots,s,0) == 0; c6 = subs(lambdaddots,s,1) == 0; 
%system solver
[a0 a1 a2 a3 a4 a5] = vpasolve([c1 c2 c3 c4 c5 c6]);

lambda = subs(lambdas);
lambdadot = diff(lambda);

pdotFun = @(t,P1,P2) double(subs(lambdadot,s,t)) * (P2 - P1);

w = @(t) pdotFun(t,theta(1),theta(2));
v = @(t) sqrt(pdotFun(t,x(1),x(2))^2 + pdotFun(t,y(1),y(2))^2);

Xdot = [v(t)*cos(X(3)) ; v(t)*sin(X(3)) ; w(t)];

end
