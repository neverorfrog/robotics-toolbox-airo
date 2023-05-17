clc; clear; close all;

%% Dati del task
P1=[1 1]; P2=[3 1]; P3=[3 3]; P4=[1 3];
% P12=[2;1]; P23=[4;2]; P34=[2;3];
P=[P1 ; P2 ; P3 ; P4];
%Istanti di visita, avró quindi una matrice 4x4
T1=0; T2=1; T3=2; T4=3;
T = [T1 T2 T3 T4]';
%Matrice di Vandermonde 
V=[T1^3 ,  T1^2 , T1 ,  1;
      T2^3 , T2^2 , T2 , 1;
      T3^3 , T3^2 , T3 , 1;
      T4^3 , T4^2 , T4 , 1];
  

%% Approccio polinomiale
%Genero la matrice dei coefficienti
t=T1:0.025:T4;
%   V=[T1^3 ,  T1^2 , T1 ,  1;
%       T2^3 , T2^2 , T2 , 1];
COE=V \ P;
% %Vettori per i coefficienti del polinomio x (A) e y (B)
A=COE(:,1); B=COE(:,2);
%Orizzonte temporale su cui generare il polinomio interpolatore

%Polinomi effettivi
X=polyval(A,t); Y=polyval(B,t); %Genero due polinomi di lunghezza A valutati negli istanti di t

figure(1); hold on; grid;
plot(X,Y)
xlabel('X'); ylabel('Y')
plot(P(:,1),P(:,2),'o')

% %% Approccio per interpolazione con curve spline
% t=T1:0.025:T4;
% xx=spline(T,P(:,1),t);
% yy=spline(T,P(:,2),t);
% 
% figure(1); hold on; grid on;
% plot(xx,yy)
% xlabel('X'); ylabel('Y')
% plot(P(:,1),P(:,2),'o')
% title(' Cubic Spline with 7 nodes')
% 
% %Grafico y-t
% figure(2); grid; hold on;
% plot(t,xx,t,yy)
% xlabel('T'); ylabel('X,Y')
% 
% %Plot dinamico
% figure(3); axis([0 5 0 5]), hold on;
% traj = [xx' yy'];
% for i = 1 : length(xx)-1
%     traj(i,3) = atan2(traj(i+1,2)-traj(i,2),traj(i+1,1)-traj(i,1));
% end
% plotRobot(traj,0.5)
