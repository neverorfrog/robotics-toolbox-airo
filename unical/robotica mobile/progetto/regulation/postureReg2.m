function Xdot = postureReg2(t,X,Xstar,k1,k2,k3)

%% Error
x = X(1); y = X(2); theta = X(3);
xstar = Xstar(1); ystar = Xstar(2); thetastar = Xstar(3);
ex = xstar - x; ey = ystar - y; etheta = angleSub(thetastar,theta);

%% Control signals
rho = sqrt(ex^2 + ey^2);
gamma = angleSub(atan2(ey,ex),theta);
delta = angleSub(gamma,etheta);

v = k1*rho*cos(gamma);
w = k2*gamma + (k1*sin(gamma)*cos(gamma)*angleAdd(gamma,k3*delta))/gamma;

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
