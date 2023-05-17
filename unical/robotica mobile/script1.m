%%
clear all;
close all;
clc;
%%
x=0:0.1:10;x=x(:);
a=2;
q=1;
yn=a*x+q;
y=yn+10*(rand(size(yn))-0.5);
%%

Phi=[x ones(size(x))];
% theta_star=(inv(Phi'*Phi)*(Phi'))*y;
theta_star=pinv(Phi)*y;
%%
figure();
hold on;grid on;
% plot(x,yn,'or','LineWidth',2);
plot(x,y,'xk','LineWidth',3);
plot(x,a*x+q,'b','LineWidth',3);
plot(x,theta_star(1)*x+theta_star(2),'m','LineWidth',3);
axis('equal');