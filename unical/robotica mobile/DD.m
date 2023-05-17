%% Differential Drive
clear; close all; clc;

%% Traiettoria percorsa dal robot generico
%Voglio fare percorrere al robot una traiettoria circolare con
%centro (Cx, Cy) con raggio R.

%Raggio e centro
R = 2;
Cx = 1; Cy = 1;
%Legge oraria - decido che sia costante
w_alpha = deg2rad(10); %rad/sec

%Scala dei tempi
T = 0.1;
Tmax = 50;
t = 0:T:Tmax;

%Vettori istante per istante della posa del robot
alpha = w_alpha*t; 
x = Cx + R*cos(alpha);
y = Cy + R*sin(alpha);

%Vettori istante per istante 
%della velocitá di rotazione
w = ones(size(t))*w_alpha;
%E di traslazione
xdot = -R*sin(alpha)*w_alpha;
ydot = R*cos(alpha)*w_alpha;
v = sqrt(xdot.^2 + ydot.^2);

%% Modello specifico del Differential Drive

r = 0.05; %raggio delle ruote di 5 centimetri
L = 0.15; %distanza tra le due ruote di 15 centimetri

%K rende possibile esprimere v e w in funzione delle due velocitá
%impresse alle ruote
K = [r/2 r/2 ; r/L -r/L];
wRwL = K \ [v ; w]; %inv(K) * [v ; w]
wL = wRwL(2,:); wR = wRwL(1,:); 

%Applico quindi i segnali wR e wL a un robot simulato
P = [Cx + R ; Cy ; pi/2]; %posa di partenza in (2,1)
for k = 1 : length(v)-1
    Pdot = [v(k)*cos(P(3,end));
                v(k)*sin(P(3,end));
                w(k)];
    %Applico la disretizzazione per trovare le posizioni P data la
    %funzione derivata Pdot, attuando praticamente un'integrazione
    %numerica
    P(: , end+1) = P(:,end) + Pdot*T;
end

figure(1); hold on; grid;
subplot(3 , 2 , [1 , 3]);
plot(x , y , "b" , "LineWidth" , 2), axis("equal");
subplot(3 , 2 , 2), title("w"); hold on; grid; plot(t,w);
subplot(3 , 2 , 4), title("v"); hold on; grid; plot(t,v);
subplot(3 , 2 , 5), title("wR"); hold on; grid; plot(t,wR);
subplot(3 , 2 , 6), title("wL"); hold on; grid; plot(t,wL);
