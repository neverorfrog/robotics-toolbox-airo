close all; clear; 

%% Points to be visited
% voronoiscript; hold on;
P1 = [5 30 pi]; P2 = [45 4 pi/2]; P3 = [0 0 pi]; P = [P1 ; P2 ; P3];

%% Control law
k1 = 1; k2 = 1; k3 = 1;

for i = 1:length(P)-1
    % controlF = @(t,X) (postureReg1(t,X,Xstar,k1,k2));
    X = P(i,:); Xstar = P(i+1,:);
    controlF = @(t,X) (postureReg2(t,X,Xstar,k1,k2,k3));
    [t,yOut] = ode45(controlF,[0 50],X);
    plotTriangle([X(1) X(2)],X(3),1); plotTriangle([Xstar(1) Xstar(2)],Xstar(3),1);
    hold on;
    plotRobot(yOut,1);
    plotTriangle([X(1) X(2)],X(3),1); plotTriangle([Xstar(1) Xstar(2)],Xstar(3),1);
end

%% Plotting
% subplot(3,1,1);
% plot(t,ones(length(t),1)*xstar,"b"); hold on; plot(t,yOut(:,1));
% 
% subplot(3,1,2);
% plot(t,ones(length(t),1)*ystar,"b"); hold on; plot(t,yOut(:,2));
% 
% subplot(3,1,3);
% plot(t,ones(length(t),1)*thetastar,"b"); hold on; plot(t,yOut(:,3));

figure(); hold on;




