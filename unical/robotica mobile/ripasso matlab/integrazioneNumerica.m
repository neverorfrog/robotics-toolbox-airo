%%
clear all;
close all;
clc;

%%
w = 0.5;
f = @(t) (sin(w*t));
int_f = @(t) (-cos(w*t)/w);
T = 1; %tempo di campionamento
Tmax = 10;
tC = 0:0.001:Tmax; %Dominio continuo
plot( tC , int_f(tC) , 'LineWidth' , 2); 
hold on; grid on;

%%
tD = [0 : T : Tmax]';
int_f_D = zeros(size(tD,1),1);
int_f_D(1) = [int_f(tD(1))]; %condizione iniziale
for k = 2 : length(tD)
    xk = int_f_D(k-1); %parto dalla condizione iniziale
    xk1 = xk + T * f(tD(k-1)); %ottengo l'area attuale in base alla precedente
    int_f_D(k) = xk1; %aggiorno il vettore dell'integrale
end

plot( tD , int_f_D ,"--r", "LineWidth" , 2);
