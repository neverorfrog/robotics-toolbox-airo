% close all; clear;

%% Motion Planning

pstart = [5 5]; pgoal = [40 40];

P = celldecomp(pstart,pgoal);
% P = voronoimap(pstart,pgoal);
% P = potentialscript(pstart,pgoal);
% P = visibilitygraph(pstart,pgoal);
% P = discretepotential(pstart,pgoal);

%% Trajectory generation
traj = r.traj;
for i = 2 : length(traj)
    traj(i,3) = atan2(traj(i,2)-traj(i-1,2),traj(i,1)-traj(i-1,1)); 
end

for i = length(traj) : -1 : 2
%     theta uguali
    while i <= length(traj) && abs(traj(i,3)-traj(i-1,3)) < 0.1 && i > 1
        traj(i-1,:) = [];
    end
end

for i = length(traj) : -1 : 2
    %distanza molto piccola tra un punto e un altro
    while i <= length(traj) && norm(traj(i,1:2)-traj(i-1,1:2)) < 0.1 && i > 1
        traj(i-1,:) = [];
    end
end

traj = traj(:,1:2);
T = linspace(0,length(traj)-1,length(traj));

global t; syms t;
[xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,tsim] = trajectoryGen(traj,T,"curves");

%Traiettoria di partenza
x = subs(xstar(1),t,0)+0.5; 
y = subs(ystar(1),t,0)+0.5; 
theta = atan((traj(2,2)-traj(1,2))/(traj(2,1)-traj(1,1)));
X = double([x  y  theta]);

% Controllo linearizzato
% a = 10; chi = 1;
% controlFunction = @(t,X) ...
%     (trajControl1(t,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,chi,a,tsim));

% Controllo nonlineare
k1 = @(vstar,wstar) (1); k3 = @(vstar,wstar) (1); k2 = 0.5;
controlFunction = @(t,X) ...
    trajControl2(t,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,k1,k2,k3,tsim);

% Feedback Linearization (per mindist é l'unico fattibile)
% k1 = 1; k2 = 1; b = 1;
% controlFunction = @(t,X) ...
%     trajControl3(t,X,xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,b,k1,k2,tsim);

[tOut,yOut] = ode45(controlFunction,[tsim(1)+0.01 tsim(end)-0.01],X);

%% Posture Regulation finale per aggiustare il tiro

% k1 = 1; k2 = 1; k3 = 1;
% X = yOut(length(yOut),:); Xstar = [pgoal,0];
% controlFunction = @(t,X) (postureReg2(t,X,Xstar,k1,k2,k3));
% % controlFunction = @(t,X) postureReg1(t,X,Xstar(1:2),k1,k2);
% [tOut,yOuttemp] = ode45(controlFunction,[0 10],X);
% yOut = [yOut ; yOuttemp];

%% Plotting

figure(2); axis([0 50 0 50]), hold on, axis equal; plotRobot(yOut,1.5);
for i = 1 : length(tsim)-1
    samples = linspace(tsim(i),tsim(i+1),20);
    plot(subs(xstar(i),t,samples),subs(ystar(i),t,samples),"ob");
end
