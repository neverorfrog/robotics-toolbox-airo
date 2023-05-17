%raggio esterno 1.85 e raggio interno 0.15
L1=0.9; L2=0.75; 
L=[L1 L2];

%Definisco quindi punto iniziale e finale del percorso e
%l'inversione cinematica per i due punti
P1=[1;1]; P2=[1.5;-0.5];
Q1 = invkin_twolink(L,P1); %coppia di variabili di giunti associata a P1
Q2 = invkin_twolink(L,P2); %coppia di variabili di giunti associata a P2

%Campionamento del percorso
dl=0.02; lambda=0:dl:1;
N=length(lambda);

%% Genero un percorso a minima distanza nello spazio dei giunti

Q = zeros(2,N); P = zeros(2,N);
for i = 1:N
    Q(:,i) =  Q1 + lambda(i) * (Q2 - Q1);
    %Ho quindi bisogno di una function che mi descrive la
    %cinematica diretta del robot per ottenere i punti nello
    %spazio cartesiano (o di lavoro)
    P(:,i) = translation(dirkin_twolink(L,Q(:,i)));
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

%Plot di come variano le variabili di giunto con lambda. Si vede
%qui come l'inversione porti a un grafico con curve, il ché é
%spiegato dal fatto che il problema é non-lineare.
Qdeg=Q*180/pi; %Le esprimo in gradi in centigradi
figure(3)
plot(lambda,Qdeg(1,:),'+',lambda,Qdeg(2,:),'*'),grid
xlabel('\lambda')
ylabel('Q [deg]')

%% Plotting
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
