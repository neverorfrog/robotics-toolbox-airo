clc; clear; close all;

%Definisco uno spazio di lavoro a forma di settore circolare con
%raggio esterno 1.85 e raggio interno 0.15
L1=0.9; L2=0.75; 
L=[L1 L2];

%Definisco quindi punto iniziale e finale del percorso
P1=[1;1];
P2=[1.5;0.3];

%Campionamento del percorso
dl=0.02; lambda=0:dl:1;
N=length(lambda);

%% Generazione percorso a minima distanza

%Genero un percorso lineare tra i due punti iniziale e finale
%prendendo N campioni su questo percorso. Il parametro lambda
%quindi é praticamente contenuto in questo vettore di campioni.
for i=1:N
    P(:,i)=P1+lambda(i)*(P2-P1); 
end

%Plot del percorso rettilineo sul piano x-y
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

%% Inversione cinematica

%Naturalmente devo invertire la cinematica per ognuno dei punti
%campioni. Ottengo quindi un vettore di 11 colonne e 2 righe, le
%cui colonne sono q1 e q2.
for i=1:N
Q(:,i)=invkin_twolink(L,P(:,i),1);
end

%Plot di come variano le variabili di giunto con lambda. Si vede
%qui come l'inversione porti a un grafico con curve, il ché é
%spiegato dal fatto che il problema é non-lineare.
Qdeg=Q*180/pi; %Le esprimo in gradi in centigradi
figure(3)
plot(lambda,Qdeg(1,:),'+',lambda,Qdeg(2,:),'*'),grid
xlabel('\lambda')
ylabel('Q [deg]')

%%
figure(4)
hold off
%Plot dei bracci del robot al variare della posizione sulla retta
for i=1:N
    q1=Q(1,i);
    q12=q1+Q(2,i);
    plot([0,L1*cos(q1)],[0,L1*sin(q1)])
    hold on
    plot([L1*cos(q1),L1*cos(q1)+L2*cos(q12)],[L1*sin(q1),L1*sin(q1)+L2*sin(q12)])
end
%Plot della retta
plot(P(1,:),P(2,:),'*'), grid
plot(P(1,:),P(2,:),'g','LineWidth',2)
xlabel('X')
ylabel('Y')
