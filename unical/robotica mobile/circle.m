clear; clc; close all;

%% Dati circonferenza
R = 2; Cx = 1; Cy = 2; a = 1; b = 2; wAlpha = deg2rad(6); 
%Costruzione circonferenza
tCirc = linspace(0,2*pi,500);
circonferenza = [Cx+R*cos(tCirc) ; Cy+R*sin(tCirc)];

%Posa iniziale del robot
theta0 = pi; alpha = theta0 - pi/2;

%% ODE45
tMax = theta0/wAlpha;
tSim = linspace(0, tMax , 50);
OdeFun = @(t,X) (KinModel(t,X,R,wAlpha));
[tOut , yOut] = ode45(OdeFun , tSim , [Cx Cy+R theta0]);

%% Plotting
plot( circonferenza(1,:) , circonferenza(2,:), "b","LineWidth" , 2); 
hold on;
axis("equal");
plotRobot(yOut,0.5);
