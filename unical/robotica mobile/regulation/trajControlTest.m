close all; clear; 
%Parto da una traiettoria e devo arrivare a una traiettoria

%% Punti della traiettoria

P1 = [2 2]; P2 = [16 17]; P3 = [20 0]; Ptemp = [P1 ; P2 ; P3];

P(1,:) = Ptemp(1,:);
%Trovo i punti di congiunzione dei tratti curvilinei
j = 2;
for i = 2 : length(Ptemp)-1
    P(j,:) = Ptemp(i-1,:)+90/100*[Ptemp(i,1)-Ptemp(i-1,1) , Ptemp(i,2)-Ptemp(i-1,2)];
    P(j+1,:) = Ptemp(i,:)+10/100*[Ptemp(i+1,1)-Ptemp(i,1) , Ptemp(i+1,2)-Ptemp(i,2)];
    j = j+2;
end

P(length(P)+1,:) = Ptemp(length(Ptemp),:);
T = linspace(0,10,length(P))';

global t; syms t;
[xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar] = trajectoryGen2(t,P,T);

% thetastar = @(t) atan2(ydotstar(t),xdotstarc(t,P(2,1),P(3,1),2));
% Xstar = @(t)(double([xstarc(t,P(2,1),P(3,1),2) ystar(t) thetastar(t)]));
% Xdotstar =@(t) (double([xdotstarc(t,P(2,1),P(3,1),2)  ydotstar(t)]));
% Xddotstar = @(t) (double([xddotstarc(t,P(2,1),P(3,1),2)  yddotstar(t)]));

% tsim1 = P(1,1) : 0.025 : P(2,1);
% tsim2 = P(2,1) : 0.025 : P(3,1);
% tsim3 = P(3,1) : 0.025 : P(4,1);
% plot(subs(xstar(1),t,tsim1),subs(ystar(1),t,tsim1)); hold on;
% plot(subs(xstar(2),t,tsim2),subs(ystar(2),t,tsim2));
% plot(subs(xstar(3),t,tsim3),subs(ystar(3),t,tsim3));

% %% Traiettoria di partenza
x = subs(xstar(1),t,0)+0.5;
y = subs(ystar(1),t,0)+0.5;
theta = (P(2,2)-P(1,2))/(P(2,1)-P(1,1))+0.2;
X = double([x  y  theta]);

%% Orizzonte temporale in base ai punti da visitare

global orizzonte; orizzonte = [];

for i = 1 : length(P)-1
    orizzonte = [orizzonte ; linspace(P(i,1)+0.005,P(i+1,1),20)'];
end

%% Controllo linearizzato

% a = 1; chi = 1;
% controlFunction = @(t,X) ...
%  (trajControl1(t,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,chi,a,P));

% yOut = [];
% global index; index = 1;
% for i = 1 : 1
%     x = double(subs(xstar(i),t,P(i,1)))-0.5;
%     y = double(subs(ystar(i),t,P(i,1)));
%     theta = (P(i+1,2)-P(i,2))/(P(i+1,1)-P(i,1));
%     X = [x y theta];
%     [tOuttemp,yOuttemp] = ode45(controlFunction,[P(i,1) P(i+1,1)],X);
%     yOut = [yOut ; yOuttemp];
%     index = index+1;
% end
% 
% %% Controllo nonlineare

k1 = @(vstar,wstar) (1); k3 = @(vstar,wstar) (1); k2 = 1;
controlFunction = @(t,X)...
(trajControl2(t,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,P,k1,k2,k3));
% 
% 
% %% Feedback Linearization
%  
% % k1 = 1; k2 = 1; b = 1;
% % controlFunction = @(t,X) (trajControl3(t,X,Xstar,Xdotstar,b,k1,k2));
% 
% %% Plotting
% 
% subplot(3,1,1);
% plot(t,xstarc(t,P(2,1),P(3,1),2),"b"); hold on; plot(t,yOut(:,1));
% 
% subplot(3,1,2);
% plot(t,ystar(t),"b"); hold on; plot(t,yOut(:,2));
% 
% subplot(3,1,3);
% plot(t,thetastar(t),"b"); hold on; plot(t,yOut(:,3));
% 
[tOuttemp,yOuttemp] = ode45(controlFunction,[1 90],X);
figure(); axis([0 20 0 20]), hold on;
plotRobot(yOuttemp,0.5);
