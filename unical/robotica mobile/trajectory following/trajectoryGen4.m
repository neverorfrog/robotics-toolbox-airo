function [xstar xdotstar xddotstar ystar ydotstar yddotstar thetastar] = ...
    trajectoryGen4(t,P,varargin)

%% Legge oraria della traiettoria

if nargin == 2 || varargin{1} == "curves"
    

%% X
syms a0 a1 a2 a3 b0 b1 b2 b3 c0 c1 c2 c3 s lambda1(s) lambda2(s);

lambda1(s) = a0*s^3 + a1*s^2 + a2*s + a3;
lambda1dot = diff(lambda1);

lambda2(s) = b0*s^3 + b1*s^2 + b2*s + b3;
lambda2dot = diff(lambda2);

lambda3(s) = c0*s^3 + c1*s^2 + c2*s + c3;
lambda3dot = diff(lambda3);

c(1) = lambda1(0) == 0; c(2) = lambda1(1) == 1;
c(3) = lambda2(1) == 0; c(4) = lambda2(2) == 1;
c(5) = lambda3(2) == 0; c(6) = lambda3(3) == 1;

c(7) = lambda1dot(0) == 0; c(8) = lambda3dot(3) == 0;
% c(9) = lambda1dot(1)*(P(2,1)-P(1,1)) == lambda2dot(1)*(P(3,1)-P(2,1));
% c(10) = lambda2dot(2)*(P(3,1)-P(2,1)) == lambda3dot(2)*(P(4,1)-P(3,1));
c(9) = lambda1dot(1) == lambda2dot(1);
c(10) = lambda2dot(2) == lambda3dot(2);
c = c(:);

coeff = vpasolve(c);
lambdax1 = subs(lambda1,coeff);
lambdax2 = subs(lambda2,coeff);
lambdax3 = subs(lambda3,coeff);
lambdax1dot = subs(lambda1dot,coeff);
lambdax2dot = subs(lambda2dot,coeff);
lambdax3dot = subs(lambda3dot,coeff);

x1 = P(1,1) + lambdax1*(P(2,1)-P(1,1));
x2 = P(2,1) + lambdax2*(P(3,1)-P(2,1));
x3 = P(3,1) + lambdax3*(P(4,1)-P(3,1));
x1dot = lambdax1dot*(P(2,1)-P(1,1));
x2dot = lambdax2dot*(P(3,1)-P(2,1));
x3dot = lambdax3dot*(P(4,1)-P(3,1));

%% Y

syms a0 a1 a2 a3 b0 b1 b2 b3 c0 c1 c2 c3 s lambda1(s) lambda2(s);

lambda1(s) = a0*s^3 + a1*s^2 + a2*s + a3;
lambda1dot = diff(lambda1);

lambda2(s) = b0*s^3 + b1*s^2 + b2*s + b3;
lambda2dot = diff(lambda2);

lambda3(s) = c0*s^3 + c1*s^2 + c2*s + c3;
lambda3dot = diff(lambda3);

c(1) = lambda1(0) == 0; c(2) = lambda1(1) == 1;
c(3) = lambda2(1) == 0; c(4) = lambda2(2) == 1;
c(5) = lambda3(2) == 0; c(6) = lambda3(3) == 1;

c(7) = lambda1dot(0) == 0; c(8) = lambda3dot(3) == 0;
% c(9) = lambda1dot(1)*(P(2,2)-P(1,2)) == lambda2dot(1)*(P(3,2)-P(2,2));
% c(10) = lambda2dot(2)*(P(3,2)-P(2,2)) == lambda3dot(2)*(P(4,2)-P(3,2));
c(9) = lambda1dot(1) == lambda2dot(1);
c(10) = lambda2dot(2) == lambda3dot(2);
c = c(:);

coeff = vpasolve(c);
lambday1 = subs(lambda1,coeff);
lambday2 = subs(lambda2,coeff);
lambday3 = subs(lambda3,coeff);
lambday1dot = subs(lambda1dot,coeff);
lambday2dot = subs(lambda2dot,coeff);
lambday3dot = subs(lambda3dot,coeff);

y1 = P(1,2) + lambday1*(P(2,2)-P(1,2));
y2 = P(2,2) + lambday2*(P(3,2)-P(2,2));
y3 = P(3,2) + lambday3*(P(4,2)-P(3,2));
y1dot = lambday1dot*(P(2,2)-P(1,2));
y2dot = lambday2dot*(P(3,2)-P(2,2));
y3dot = lambday3dot*(P(4,2)-P(3,2));

% Polinomio interpolante
syms d0 d1 d2 d3 x ypoli(x);
ypoli(x) = d0*x^3 + d1*x^2 + d2*x + d3;
ypolidot = diff(ypoli);

c1 = ypoli(P(2,1)) == P(2,2); c2 = ypoli(P(3,1)) == P(3,2);
c3 = ypolidot(P(2,1)) == y2dot(2);
c4 = ypolidot(P(3,1)) == y2dot(3);
coeff = vpasolve([c1 c2 c3 c4]);
ypoli = subs(ypoli,coeff); ypoli(s) = subs(ypoli,x,x2);
ypolidot = subs(ypolidot,coeff); ypolidot(s) = subs(ypolidot,x,x2);


for i = 1 : 3
    tsim(:,i) = i-1:0.01:i;
end
figure(1), hold on;
plot(subs(x1,s,tsim(:,1)),subs(y1,s,tsim(:,1)));
plot(subs(x2,s,tsim(:,2)),subs(ypoli,s,tsim(:,2)));
plot(subs(x3,s,tsim(:,3)),subs(y3,s,tsim(:,3)));


%% Generazione traiettoria

for i = 1 : 2 : length(P)-1
    
    x1 = P(i,1); y1 = P(i,2);
    x2 = P(i+1,1); y2 = P(i+1,2);
    
    xstar{i} = x1 + (lambda(tchange(i):tchange(i+1)) - (i-1)) * (x2-x1);
    xdotstar{i} = (lambdadot(tchange(i):tchange(i+1)) - (i-1)) * (x2-x1);
    xddotstar{i} = (lambdaddot(tchange(i):tchange(i+1)) - (i-1)) * (x2-x1);
    ystar{i} = y1 + (lambda(tchange(i):tchange(i+1)) - (i-1)) * (y2-y1);
    ydotstar{i}  = (lambdadot(tchange(i):tchange(i+1)) - (i-1)) * (y2-y1);
    yddotstar{i} = (lambdaddot(tchange(i):tchange(i+1)) - (i-1)) * (y2-y1);
    thetastar{i} = atan2(ydotstar{i},xdotstar{i});
    
    %Tratto curvo
    if i < length(P) - 1
        
        x3 = P(i+2,1); y3 = P(i+2,2);
        xstar{i+1} = x2 + (lambda(tchange(i+1):tchange(i+2)) - (i)) * (x3-x2);
        xdotstar{i+1} = (lambdadot(tchange(i+1):tchange(i+2)) - (i)) * (x3-x2);
        xddotstar{i+1} = (lambdaddot(tchange(i+1):tchange(i+2)) - (i)) * (x3-x2);
        
        syms a0 a1 a2 a3 x(t) xdot(t) xddot(t) ys(t);
        ys = a0*x^3 + a1*x^2 + a2*x + a3;
        ysdot = diff(ys);
        ysddot = diff(ysdot);
        %condizioni
        c1 = subs(ys(x2),x,t) == y2;
        c2 = subs(ys(x3),x,t) == y3;
        c3 = subs(ysdot(x2),x,t) == (lambdadot(tchange(i+1)) - i)*(y3-y2);
        c4 = subs(ysdot(x3),x,t) == (lambdaddot(tchange(i+1)) - i)*(y3-y2);
        %soluzione
            [a0 a1 a2 a3] = vpasolve([c1,c2,c3,c4],[a0 a1 a2 a3]);
            ystartemp = subs(ys); ydotstartemp = subs(ysdot); yddotstartemp = subs(ysddot);
            
            ytemp = []; ydottemp = []; yddottemp = [];
            xtemp = xstar{2};
            for j = 1 : length(xstar{i+1})
                ytemp = [ytemp ; subs(ystartemp,x,xtemp(j))];
                ydottemp = [ydottemp ; subs(ydotstartemp,x,xtemp(j))];
                yddottemp = [ydottemp ; subs(yddotstartemp,x,xtemp(j))];
            end
                
                
            ystar(i+1,1) = subs(ystartemp,x,xstar(i+1));
            ydotstar(i+1,1) = subs(ydotstartemp,x,xstar(i+1));
            yddotstar(i+1,1) = subs(yddotstartemp,x,xstar(i+1));
            end
            
            thetastar(i+1,1) = atan2(ydotstar(i+1),xdotstar(i+1));
        end
        
end
    
end
