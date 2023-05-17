function Xdot = trajControl1(tsim,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,chi,a,tv)

%% Estrazione dati dalla traiettoria da inseguire

% Controllo l'istante attuale e di conseguenza estraggo la parte di traiettoria che
% concerne il tratto da percorrere
for i = 1 : length(tv)-1
    if tsim >= tv(i) && tsim < tv(i+1)
        xstar = xstar(i); ystar = ystar(i); thetastar = thetastar(i);
        xdotstar = xdotstar(i); ydotstar = ydotstar(i);
        xddotstar = xddotstar(i); yddotstar = yddotstar(i);
    end
end

global t;
x = X(1); y = X(2); theta = X(3);
xstar = subs(xstar,t,tsim); ystar = subs(ystar,t,tsim); thetastar = subs(thetastar,t,tsim);
xdotstar = subs(xdotstar,t,tsim); ydotstar = subs(ydotstar,t,tsim);
xddotstar = subs(xddotstar,t,tsim); yddotstar = subs(yddotstar,t,tsim);


%% Error

ex = cos(theta)*(xstar-x) + sin(theta)*(ystar-y);
ey = -sin(theta)*(xstar-x) + cos(theta)*(ystar-y);
etheta = angleSub(thetastar,theta);

%% Control signals

vstar = sqrt(xdotstar^2 + ydotstar^2);
wstar = (yddotstar*xdotstar - ydotstar*xddotstar) / (xdotstar^2 + ydotstar^2);

k1 = 2*chi*a;
k3 = k1;
k2 = (a^2 - (wstar)^2) / (vstar);

v = double(vstar*cos(etheta) + k1*ex);
w = double(wstar + k2*ey + k3*etheta);

%% Modello specifico del Differential Drive

r = 0.05; %raggio delle ruote di 5 centimetri
L = 0.15; %distanza tra le due ruote di 15 centimetri

%K rende possibile esprimere v e w in funzione delle due velocitá
%impresse alle ruote
K = [r/2 r/2 ; r/L -r/L];
wRwL = K \ [v ; w];
Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);

% Xdot = [v*cos(theta) ; v*sin(theta) ; w];
end
