function [xstar xdotstar xddotstar ystar ydotstar yddotstar thetastar] = ...
    trajectoryGen3(t,P,T,varargin)

%Se devo visitare n punti, genereró 6 vettori di n componenti ciascuno. Esse
%costituiscono i polinomi che generano il percorso tra ogni coppia di punti, che sia
%curvilineo o a distanza minima.

%% Legge oraria della traiettoria

if nargin == 3 || varargin{1} == "curves"
    
   %lambda simbolico
    n = size(P,1) - 1;
    
    syms("a",[n 4]); syms s(ts) t; syms("lambdaxsym(ts)",[n 1]);
    smatrix = repmat([s^3,s^2,s,1],[n,1]);
    lambdaxsym = sum(a.*smatrix,2); 
    lambdaxdotsym = simplify(sum(diff(lambdaxsym),2)); 
    lambdaxddotsym = simplify(sum(diff(lambdaxdotsym),2));
    
    syms("b",[n 4]); syms s(ts) t; syms("lambdaysym(ts)",[n 1]);
    smatrix = repmat([s^3,s^2,s,1],[n,1]);
    lambdaysym = sum(b.*smatrix,2); 
    lambdaydotsym = simplify(sum(diff(lambdaysym),2)); 
    lambdayddotsym = simplify(sum(diff(lambdaydotsym),2));
    
    for i = 1 : n+1
        lambdaxt(:,:,i) = lambdaxsym(T(i)); 
        lambdaxtdot(:,:,i) = lambdaxdotsym(T(i));
        lambdaxtddot(:,:,i) = lambdaxddotsym(T(i));
        
        lambdayt(:,:,i) = lambdaysym(T(i)); 
        lambdaytdot(:,:,i) = lambdaydotsym(T(i));
        lambdaytddot(:,:,i) = lambdayddotsym(T(i));
    end
    
    %Inizio e fine
    %Il primo indice indica l'ennesima lambda, il terzo l'istante attuale
    c(1,1) = subs(lambdaxt(1,1,1),s,ts) == 0; c(2,1) = subs(lambdaxt(n,1,n+1),s,ts) == 1;
    c(3,1) = subs(lambdaxtdot(1,1,1),s,ts) == 0; c(4,1) = subs(lambdaxtddot(1,1,1),s,ts) == 0;
    
    %Intermedie
    j = 1;
    for i = 5 : 4 : 4*n
        c(i,1) = subs(lambdaxt(j,1,j+1),s,ts) == 1; c(i+1,1) = subs(lambdaxt(j+1,1,j+1),s,ts) == 0;
        c(i+2,1) = subs(lambdaxtdot(j,1,j+1),s,ts) == subs(lambdaxtdot(j+1,1,j+1),s,ts);
        c(i+3,1) = subs(lambdaxtddot(j,1,j+1),s,ts) == subs(lambdaxtddot(j+1,1,j+1),s,ts);
        j = j+1;
    end
    
    coefficients = vpasolve(c);
    lambdax = subs(lambdaxsym,coefficients); 
    lambdaxdot = diff(lambdax); lambdaxddot = diff(lambdaxdot);
    
    
    %Inizio e fine
    %Il primo indice indica l'ennesima lambda, il terzo l'istante attuale
    c(1,1) = subs(lambdayt(1,1,1),s,ts) == 0; c(2,1) = subs(lambdayt(n,1,n+1),s,ts) == 1;
    c(3,1) = subs(lambdaytdot(1,1,1),s,ts) == 0; c(4,1) = subs(lambdaytddot(1,1,1),s,ts) == 0;
    
    %Intermedie
    j = 1;
    for i = 5 : 4 : 4*n
        c(i,1) = subs(lambdayt(j,1,j+1),s,ts) == 1; c(i+1,1) = subs(lambdayt(j+1,1,j+1),s,ts) == 0;
        c(i+2,1) = subs(lambdaytdot(j,1,j+1),s,ts) == subs(lambdaytdot(j+1,1,j+1),s,ts);
        c(i+3,1) = subs(lambdaytddot(j,1,j+1),s,ts) == subs(lambdaytddot(j+1,1,j+1),s,ts);
        j = j+1;
    end
    
    coefficients = vpasolve(c);
    lambday = subs(lambdaysym,coefficients); 
    lambdaydot = diff(lambday); lambdayddot = diff(lambdaydot);
    
    %% Generazione traiettoria
    
    for i = 1 : n
        lambdaxt(:,:,i) = subs(lambdax,s,t+1-i);
        lambdayt(:,:,i) = subs(lambday,s,t+1-i);
    end
    lambdaxtdot = diff(lambdaxt); lambdaxtddot = diff(lambdaxtdot);
    lambdaytdot = diff(lambdayt); lambdaytddot = diff(lambdaytdot);
    
    xstar = sym([]); xdotstar = sym([]); xddotstar = sym([]);
    ystar = sym([]); ydotstar = sym([]); yddotstar = sym([]);
    
    for i = 1 : length(P)-1
        
        x1 = P(i,1); y1 = P(i,2);
        x2 = P(i+1,1); y2 = P(i+1,2);
        
        xstar(i,1) = x1 + lambdaxt(i,1,i) * (x2-x1);
        xdotstar(i,1) = lambdaxtdot(i,1,i) * (x2-x1);
        xddotstar(i,1) = lambdaxtddot(i,1,i) * (x2-x1);
        ystar(i,1) = y1 + lambdayt(i,1,i) * (y2-y1);
        ydotstar(i,1) = lambdaytdot(i,1,i) * (y2-y1);
        yddotstar(i,1) = lambdaytddot(i,1,i) * (y2-y1);
        thetastar(i,1) = atan2(ydotstar(i),xdotstar(i));
    end

elseif varargin{1} == "mindist"    
    
end
