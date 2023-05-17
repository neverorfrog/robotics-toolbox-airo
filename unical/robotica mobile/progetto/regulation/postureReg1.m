function Xdot = postureReg1(t,X,Xstar,k1,k2)

%% Error
x = X(1); y = X(2); theta = X(3);
xstar = Xstar(1); ystar = Xstar(2);
ex = xstar - x; ey = ystar - y;

%% Control signals
v = k1 * (ex*cos(theta) + ey*sin(theta));

if norm([ex ; ey]) <= 0.01
    w = 0;
else
    w = k2 * (angleSub(atan2(ey,ex),theta));
end

%% Modello specifico del Differential Drive

r = 0.05; %raggio delle ruote di 5 centimetri
L = 0.15; %distanza tra le due ruote di 15 centimetri

%K rende possibile esprimere v e w in funzione delle due velocitá
%impresse alle ruote
K = [r/2 r/2 ; r/L -r/L];
wRwL = K \ [v ; w]; %inv(K) * [v ; w]
Xdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL);

%Possiamo dire che il sistema di controllo é a ciclo chiuso perché questa v dipende a
%sua volta da ex,ey,etheta (in cui compare x, ovvero l'evoluzione della posizione del
%robot mobile), per cui stiamo chiudendo l'anello di feedback
% Xdot = [v*cos(theta) ; v*sin(theta) ; w];

end

