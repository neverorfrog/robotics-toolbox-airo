function Xdot = trajControl3(tsim,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,b,k1,k2,tv)

%% Estrazione dati dalla traiettoria da inseguire

% Controllo l'istante attuale e di conseguenza estraggo la parte di traiettoria che
% concerne il tratto da percorrere
for i = 1 : length(tv)-1
    if tsim >= tv(i) && tsim < tv(i+1)
        xstar = xstar(i); ystar = ystar(i); thetastar = thetastar(i);
        xdotstar = xdotstar(i); ydotstar = ydotstar(i);
        break;
    end
end
    
global t;

x = X(1); y = X(2); theta = X(3);
xstar = subs(xstar,t,tsim); ystar = subs(ystar,t,tsim); thetastar = subs(thetastar,t,tsim);
xdotstar = subs(xdotstar,t,tsim); ydotstar = subs(ydotstar,t,tsim);

xb = x + b*cos(theta);
yb = y + b*sin(theta);

%% Control signals

invT = [cos(theta) sin(theta) ; -sin(theta)/b cos(theta)/b];

u1u2 = [xdotstar + k1*(xstar - xb) ; ydotstar + k2*(ystar - yb)];
vw = invT*u1u2;

v = double(vw(1));
w = double(vw(2));

%% Modello specifico del Differential Drive

r = 0.05; %raggio delle ruote di 5 centimetri
L = 0.15; %distanza tra le due ruote di 15 centimetri

%K rende possibile esprimere v e w in funzione delle due velocitá
%impresse alle ruote
K = [r/2 r/2 ; r/L -r/L];
wRwL = K \ [v ; w]; %inv(K) * [v ; w]
Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);

% Xdot = [v*cos(theta) ; v*sin(theta) ; w];

end
