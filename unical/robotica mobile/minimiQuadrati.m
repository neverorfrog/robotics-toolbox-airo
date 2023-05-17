%%
clear; close all; clc;

%% Retta
%dominio
x = 0:0.1:10; x = x(:); %ottengo cosí un vettore colonna
%coefficienti
a = 2;
q = 1;
%equazione della retta nominale e di n punti random
yn = a*x + q;
y = yn + 10*rand(size(yn));

%% Minimi quadrati

Phi = [x ones(size(x))];
%Mi serve per forza la pseudo-inversa perché phi non é quadrata
theta_star = ((Phi'*Phi) \ Phi' ) * y;
%oppure
theta_star = pinv(Phi) * y;

%% Plot
figure(1);
hold on; grid;
plot(x,y,"or","LineWidth",2);
plot(x, theta_star(1)*x + theta_star(2) , "b" , "LineWidth" , 2); 
axis("equal");


