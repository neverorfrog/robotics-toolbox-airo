function [xstar, xdotstar, xddotstar, ystar, ydotstar, yddotstar, thetastar, tsim] = trajectoryGen(P,T,varargin)

% Generazione di traiettorie:
% 1. Per tratti a minima distanza con profilo di velocitá
%     trapezoidale uniti da polinomi cubici con derivata iniziale e finale continua con i
%     tratti rettilinei (tranne che a inizio e fine, in cui la velocitá é nulla).
% 2. Per tratti a minimi distanza con fermata a ogni punto

global t; syms t;

if nargin == 2 || varargin{1} == "curves"
    
    bt = 0.1; %blend time
    ystar = sym([]); xstar = sym([]);
    ydotstar = sym([]); xdotstar = sym([]);
    yddotstar = sym([]); xddotstar = sym([]);
    thetastar = sym([]);
    ystar(1,1) = P(1,2); ydotstar(1,1) = 0; ydotstar(2*length(P)+1,1) = 0;
    xstar(1,1) = P(1,1); xdotstar(1,1) = 0; xdotstar(2*length(P)+1,1) = 0;
    
    j = 2;
    for i = 1 : length(P)
        
        syms a0 a1 a2 a3 b0 b1 b2 b3 ysym;
        
        if i < length(P)
            
            %tratto rettilineo dal punto i al punto i+1
            t1 = T(i); t2 = T(i+1); deltat = t2-t1;
            tr1 = t1+bt; tr2 = t2-bt;
            y1 = P(i,2); y2 = P(i+1,2);
            x1 = P(i,1); x2 = P(i+1,1);
            ydotstar(j+1,1) = (y2-y1)/deltat + 0.00001;
            xdotstar(j+1,1) = (x2-x1)/deltat + 0.00001;
            thetastar(j+1,1) = atan2(ydotstar(j+1),xdotstar(j+1));
            ystar(j+1,1) = ydotstar(j+1)*(t-t1) + y1;
            xstar(j+1,1) = xdotstar(j+1)*(t-t1) + x1;
            yddotstar(j+1,1) = 0; xddotstar(j+1,1) = 0;
            
        end
        
        %blend sul punto i
        if i == length(P)
            t1 = T(i); tr1 = t1+bt;
        end
        tb1 = t1-bt; tb2 = tr1;
        tsim(j-1,1) = tb1; tsim(j,1) = tb2;
        ysym = a0*t^3 + a1*t^2 + a2*t + a3;
        xsym = b0*t^3 + b1*t^2 + b2*t + b3;
        ysymdot = diff(ysym); xsymdot = diff(xsym);
        ysymddot = diff(ysymdot); xsymddot = diff(xsymdot);
        
        %sono all'ultimo blend
        if i == length(P)
            tr2 = tr2+1; t2 = t2+1;
        end
        
        %condizioni da rispettare su derivate e posizioni
        cy(1) = subs(ysym,t,tb1) == subs(ystar(j-1),t,tr2-1);
        cy(2) = subs(ysym,t,tb2) == subs(ystar(j-1),t,t2-1)+bt*ydotstar(j+1);
        cx(1) = subs(xsym,t,tb1) == subs(xstar(j-1),t,tr2-1);
        cx(2) = subs(xsym,t,tb2) == subs(xstar(j-1),t,t2-1)+bt*xdotstar(j+1);
        
        cy(3) = subs(ysymdot,t,tb1) == ydotstar(j-1);
        cy(4) = subs(ysymdot,t,tb2) == ydotstar(j+1);
        cx(3) = subs(xsymdot,t,tb1) == xdotstar(j-1);
        cx(4) = subs(xsymdot,t,tb2) == xdotstar(j+1);
        
        coeffy = vpasolve(cy); coeffx = vpasolve(cx);
        %traiettoria polinomiale
        ystar(j,1) = subs(ysym,coeffy); xstar(j,1) = subs(xsym,coeffx);
        ydotstar(j,1) = subs(ysymdot,coeffy); xdotstar(j,1) = subs(xsymdot,coeffx);
        yddotstar(j,1) = subs(ysymddot,coeffy); xddotstar(j,1) = subs(xsymddot,coeffx);
        thetastar(j,1) = atan2(ydotstar(j,1),xdotstar(j,1)); 
        
        j = j+2;
    end
    
    %elimino i tratti fittizi
    xstar(1) = []; ystar(1) = []; thetastar(1) = [];
    xdotstar(1) = []; ydotstar(1) = []; 
    xdotstar(length(xdotstar)) = []; ydotstar(length(ydotstar)) = [];
    xddotstar(1) = []; yddotstar(1) = [];
    
    
%% traiettoria per spezzate senza polinomi di blending

elseif varargin{1} == "mindist"
    
    %Initial Position and Orientation
    X(1,:) = [P(1,:) pi]; traj = X;
    %Robot path
    for i = 2 : 2 : length(P)*2 - 1
        j = i/2+1;
        thetaTraj = atan2(P(j,2)-P(j-1,2),P(j,1)-P(j-1,1));
        X(i,:) = [P(j-1,:) thetaTraj];
        X(i+1,:) = [P(j,:) thetaTraj];
    end
    tsim = linspace(T(1),T(length(T)),length(X))';
    
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
    lambdaddot = diff(lambdadot);
    
    for i = 1 : length(X)-1
        
        %Points to be visited in each iteration
        x = X(i:i+1,1); y = X(i:i+1,2); theta = X(i:i+1,3);
        x1 = x(1); y1 = y(1); theta1 = theta(1);
        x2 = x(2); y2 = y(2); theta2 = theta(2);
        t1 = tsim(i); t2 = tsim(i+1);
        
        xstar(i,1) = x1 + subs(lambda,s,(t-t1)/(t2-t1)) * (x2-x1);
        xdotstar(i,1) = subs(lambdadot,s,(t-t1)/(t2-t1)) * (x2-x1) / (t2 - t1);
        xddotstar(i,1) = subs(lambdaddot,s,(t-t1)/(t2-t1)) * (x2-x1) / (t2 - t1)^2;
        ystar(i,1) = y1 + subs(lambda,s,(t-t1)/(t2-t1)) * (y2-y1);
        ydotstar(i,1) = subs(lambdadot,s,(t-t1)/(t2-t1)) * (y2-y1) / (t2 - t1);
        yddotstar(i,1) = subs(lambdaddot,s,(t-t1)/(t2-t1)) * (y2-y1) / (t2 - t1)^2;
        thetastar(i,1) = theta1 + subs(lambda,s,(t-t1)/(t2-t1)) * (theta2-theta1);
    end    
end
