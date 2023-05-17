close all; clear;

%% Trajectory planning

%Points to be visited
P1 = [5 3]; P2 = [6 15]; P3 = [20 0]; P = [P1 ; P2 ; P3];

%Initial Position and Orientation
X(1,:) = [P(1,:) pi]; traj = X;
%Robot path
for i = 2 : 2 : length(P)*2 - 1
    j = i/2+1;
    thetaTraj = atan2(P(j,2)-P(j-1,2),P(j,1)-P(j-1,1));
    X(i,:) = [P(j-1,:) thetaTraj];
    X(i+1,:) = [P(j,:) thetaTraj];
end

%% Simulation

traj = []; tOut = [0];
for i = 1 : length(X)-1
    
    %Points to be visited in each iteration
    x = X(i:i+1,1); y = X(i:i+1,2); theta = X(i:i+1,3);
    
    %Movement
%     odeFun = @(t,X) trapezoidalVel(t,X,x,y,theta);
    odeFun = @(t,X) polynomialP2P(t,X,x,y,theta);
    [tOutTemp,trajTemp] = ode45(odeFun,[0 1],[x(1) y(1) theta(1)]);
    traj = [traj ; trajTemp];
    tOut = [tOut ; tOut(length(tOut))+tOutTemp];
end

figure(1); axis([0 20 0 20]),hold on;
plotRobot(traj,0.5);
figure(2);  tOut(1) = [];
plot(tOut,traj(:,1));

