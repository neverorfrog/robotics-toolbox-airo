clc; clear; close all;

%Definisco uno spazio di lavoro a forma di settore circolare con
%raggio esterno 1.85 e raggio interno 0.15
L1=0.9; L2=0.75; 
L=[L1 L2];

%Definisco quindi punto iniziale e finale del percorso
P1=[0;1];
Q1=invkin_twolink(L,P1);
P2=[1.5;0];
Q2=invkin_twolink(L,P2);

%Campionamento del percorso
dl=0.01; lambda=0:dl:1;
N=length(lambda);

%%
%Generazione del percorso a minima distanza
%Genero un percorso che é praticamente un arco di circonferenza

r = 0.9;
theta1deg = 0;
theta2deg = 360;
theta1 = theta1deg*pi/180;
theta2 = theta2deg*pi/180;
P = zeros(2,N); %punti della curva parametrizzata
%Riempio il vettore
for i = 1:N
    theta(i) = theta1 + lambda(i) * (theta2 - theta1);
    px = cos(theta(i)) * r;
    py = sin(theta(i)) * r;
    P(:,i) = [px ; py];
end

%Plot del percorso sul piano x-y
figure(1)
plot(P(1,:),P(2,:),'*'), grid
xlabel('X')
ylabel('Y')
hold on
plot(P(1,:),P(2,:),'g','LineWidth',2)
hold off

%Plot del percorso rettilineo sul piano lambda-P. Mi indica
%quindi la variazione del punto P al variare del parametro.
figure(2)
plot(lambda,P(1,:),'*',lambda,P(2,:),'+'),grid %Ho due grafici in uno praticamente
xlabel('\lambda')
ylabel('Px; Py')

%%
%Inversione cinematica

%Naturalmente devo invertire la cinematica per ognuno dei punti
%campioni. Ottengo quindi un vettore di 11 colonne e 2 righe, le
%cui colonne sono q1 e q2.
for i=1:N
    Q(:,i)=invkin_twolink(L,P(:,i),1);
end

%Plot di come variano le variabili di giunto con lambda. Si vede
%qui come l'inversione porti a un grafico con curve, il ché é
%spiegato dal fatto che il problema é non-lineare.
Qdeg=Q*180/pi; %Le esprimo in gradi
figure(3)
plot(lambda,Qdeg(1,:),'+',lambda,Qdeg(2,:),'*');
grid; legend("q1","q2");
xlabel('\lambda')
ylabel('Q [deg]')

%%
figure(4)
hold off
%Plot dei bracci del robot al variare della posizione sulla semicirconferenza
for i=1:N
    q1=Q(1,i);
    q12=q1+Q(2,i);
    plot([0,L1*cos(q1)],[0,L1*sin(q1)])
    hold on
    plot([L1*cos(q1),L1*cos(q1)+L2*cos(q12)],[L1*sin(q1),L1*sin(q1)+L2*sin(q12)])
end
%Plot della semicirconferenza
plot(P(1,:),P(2,:),'*'), grid
plot(P(1,:),P(2,:),'g','LineWidth',2)
xlabel('X')
ylabel('Y')

