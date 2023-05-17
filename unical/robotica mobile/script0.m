%% 
clear all;
close all;
clc;
%%
% T=0.5;
% Tmax=36;
% t=0:T:Tmax;
%%
R=2;
w_alpha=deg2rad(6);
Cx=1;
Cy=2;
tMax = pi/2/w_alpha;
tPause = 0.05;
samples = tMax/tPause;
t = linspace(0,tMax , 20*15);
%%
r=0.05;
L=0.15;
K_wRwL_2_vw=[
    r/2 r/2;
    r/L -r/L
    ];
%%
alpha_fun=@(t)(w_alpha*t);
x_circonferenza=@(t)(Cx+R*cos(alpha_fun(t)));
y_circonferenza=@(t)(Cy+R*sin(alpha_fun(t)));


xdot_circonferenza=@(t)(-R*sin(alpha_fun(t))*w_alpha);
ydot_circonferenza=@(t)( R*cos(alpha_fun(t))*w_alpha);

w_fun=@(t)(w_alpha);
v_fun=@(t)(sqrt(xdot_circonferenza(t).^2+ydot_circonferenza(t).^2));

model=@(x,v_fun,w_fun,t)([v_fun(t)*cos(x(3));v_fun(t)*sin(x(3));w_fun(t)]);
f=@(t,x)(model(x,v_fun,w_fun,t));

model_wRwL=@(x,wRwL)([cos(x(3)) 0;sin(x(3)) 0;0 1]*K_wRwL_2_vw*wRwL);
f_wRwL=@(t,x)(model_wRwL(x,K_wRwL_2_vw\[v_fun(t);w_fun(t)]));


[t,yOut]=ode45(f,t,[Cx+R;Cy;pi/2]);
% [t,Y]=ode45(f_wRwL,t,[Cx+R;Cy;pi/2]);

% %%
% figure();
% subplot(321);hold on;grid on;plot(t,Y(:,1),'LineWidth',2);
% subplot(323);hold on;grid on;plot(t,Y(:,2),'LineWidth',2);
% subplot(325);hold on;grid on;plot(t,Y(:,3),'LineWidth',2);
% subplot(3,2,[2 4]);hold on;grid on;plot(Y(:,1),Y(:,2),'LineWidth',2);

tCirc = linspace(0,2*pi,500);
circonferenza = [Cx+R*cos(tCirc) ; Cy+R*sin(tCirc)];
plot( circonferenza(1,:) , circonferenza(2,:), "b","LineWidth" , 2),  axis("equal","manual"); 
hold on;
plot_vehicle(yOut);


