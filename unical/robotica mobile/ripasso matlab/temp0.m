clear; close all; clc;

%% Prodotto scalare tra vettori a e b
a = [1;3;4;6;7];
b = [2;3;5;8;9];

q = a' * b;

%% Matrice M per vettore a
M = rand(7,5);
row = size(M,1);
col = size(M,2);

%Se ho una matrice mxn e un vettore nx1, il prodotto fra i due
%consisterá in un vettore mx1
c = zeros(row,1);
%Incolonno in c i prodotti scalari tra righe di M e a
for i = 1:row
    for j = 1:col
        c(i) = c(i) + M(i,j)*a(j);
    end
end

%OPPURE
c1 = zeros(row,1);
for i = 1:row
    c1(i) = M(i,:)*a;
end

%OPPURE
c2 = M*a;

