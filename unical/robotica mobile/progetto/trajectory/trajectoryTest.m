% clear; close all;

global t;
P = r.traj;
for i = 1 : length(P)-1
    P(i,3) = atan2(P(i+1,2)-P(i,2), P(i+1,1)-P(i,1));
end
T = linspace(0,length(P)-1,length(P));

[xstar,xdotstar,xddotstar,ystar,ydotstar,yddotstar,thetastar,tsim] = trajectoryGen(P,T,"mindist");

yOut = []; tOut = [];

for i = 1 : length(xstar)
    ttemp = linspace(tsim(i),tsim(i+1),20)';
    tOut = [tOut ; ttemp];
    yOuttemp = [subs(xstar(i),t,ttemp) subs(ystar(i),t,ttemp) subs(thetastar(i),t,ttemp)];
    yOut = [yOut ; yOuttemp];
    
    %Posizioni
    figure(2); 
    subplot(3,1,1); plot(ttemp,subs(xstar(i),t,ttemp),"r"); hold on; title("X");
    subplot(3,1,2); plot(ttemp,subs(ystar(i),t,ttemp),"b"); hold on; title("Y");
    subplot(3,1,3); plot(ttemp,subs(thetastar(i),t,ttemp),"g"); hold on; title("\theta");
    
    %Velocitá
    figure(3); 
    subplot(2,1,1); plot(ttemp,subs(xdotstar(i),t,ttemp),"r"); hold on; title("Xdot");
    subplot(2,1,2); plot(ttemp,subs(ydotstar(i),t,ttemp),"b"); hold on; title("Ydot");
end

yOut = double(yOut);

%Plot dinamico
figure(4); axis([0 50 0 50]), hold on; plotRobot(yOut);

