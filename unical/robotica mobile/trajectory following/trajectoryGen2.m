function [xstar xdotstar xddotstar ystar ydotstar yddotstar thetastar] = ...
    trajectoryGen2(t,P,T,varargin)

%Se devo visitare n punti, genereró 6 vettori di n componenti ciascuno. Esse
%costituiscono i polinomi che generano il percorso tra ogni coppia di punti, che sia
%curvilineo o a distanza minima.

%% Legge oraria della traiettoria

if nargin == 3 || varargin{1} == "curves"
    
    %Matrice simbolica di lambda
    syms("a",[size(P,1) 4]);
    n = size(P,1) - 1;
    for i = 1 : n
        lambdas(i,1) = a(i,1)*t^3 + a(i,2)*t^2 +a(i,3)*t + a(i,4);
    end
    lambdasdot = diff(lambdas);
    lambdasddot = diff(lambdasdot);
    
    %Inizio e fine
    c(1,1) = subs(lambdas(1),t,P(1,1)) == 0;
    c(2,1) = subs(lambdas(n),t,P(n+1,1)) == 1;
    c(3,1) = subs(lambdasdot(1),t,P(1,1)) == 0;
    c(4,1) = subs(lambdasdot(n),t,P(n+1,1)) == 0;
    
    %Intermedie
    j = 1;
    for i = 5 : 4 : 4*length(lambdas)
        c(i,1) = subs(lambdas(j),t,P(j+1,1)) == 1;
        c(i+1,1) = subs(lambdas(j+1),t,P(j+1,1)) == 0;
        c(i+2,1) = subs(lambdasdot(j),t,P(j+1,1)) == subs(lambdasdot(j+1),t,P(j+1,1));
        c(i+3,1) = subs(lambdasddot(j),t,P(j+1,1)) == subs(lambdasddot(j+1),t,P(j+1,1));
        j = j+1;
    end
    
    coeffstruct = vpasolve(c);
    coeffcell = struct2cell(coeffstruct);
    
    for i = 4 : 4 : length(coeffcell)
        lambda(i/4,1) = ...
            double(coeffcell{i-3})*t^3+double(coeffcell{i-2})*t^2+double(coeffcell{i-1})*t+double(coeffcell{i});
    end
    lambdadot = diff(lambda); lambdaddot = diff(lambdadot);
    
    %% Generazione traiettoria
    
    xstar = sym([]); xdotstar = sym([]); xddotstar = sym([]);
    ystar = sym([]); ydotstar = sym([]); yddotstar = sym([]);
    
    for i = 1 : 2 : length(P)-1
        
        x1 = P(i,1); y1 = P(i,2);
        x2 = P(i+1,1); y2 = P(i+1,2);
        
        xstar(i,1) = x1 + lambda(i) * (x2-x1);
        xdotstar(i,1) = lambdadot(i) * (x2-x1);
        xddotstar(i,1) = lambdaddot(i) * (x2-x1);
        ystar(i,1) = y1 + lambda(i) * (y2-y1);
        ydotstar(i,1) = lambdadot(i) * (y2-y1);
        yddotstar(i,1) = lambdaddot(i) * (y2-y1);
        thetastar(i,1) = atan2(ydotstar(i),xdotstar(i));
        
        %Tratto curvo
        if i < length(P) - 1
            
            x3 = P(i+2,1); y3 = P(i+2,2);
            x4 = P(i+3,1); y4 = P(i+3,2);
            xstar(i+1,1) = x2 + lambda(i+1) * (x3-x2);
            xdotstar(i+1,1) = lambdadot(i+1) * (x3-x2);
            xddotstar(i+1,1) = lambdaddot(i+1) * (x3-x2);
            
            syms a0 a1 a2 a3 x(t) xdot(t) xddot(t) ys(t);
            ys = a0*x^3 + a1*x^2 + a2*x + a3;
            ysdot = diff(ys);
            ysddot = diff(ysdot);
            %condizioni
            c1 = subs(ys(x2),x,xstar(i+1)) == y2;
            c2 = subs(ys(x3),x,xstar(i+1)) == y3;
            c3 = subs(ysdot(x2),x,xstar(i+1)) == (y2-y1)/(x2-x1);
            c4 = subs(ysdot(x3),x,xstar(i+1)) == (y4-y3)/(x4-x3);
            %soluzione
            [a0 a1 a2 a3] = vpasolve([c1,c2,c3,c4],[a0 a1 a2 a3]);
            ystartemp = subs(ys); ydotstartemp = subs(ysdot); yddotstartemp = subs(ysddot);
            
            ystar(i+1,1) = subs(ystartemp,x,xstar(i+1));
            ydotstar(i+1,1) = subs(ydotstartemp,x,xstar(i+1));
            yddotstar(i+1,1) = subs(yddotstartemp,x,xstar(i+1));
            
            thetastar(i+1,1) = atan2(ydotstar(i+1),xdotstar(i+1));
        end
        
    end

elseif varargin{1} == "mindist"    
    
end
