function [X,Y,grid,obstacles] = ostacoli()

% Genero una mappa di ostacoli che sistemo in una griglia binaria di occupazione
% dello spazio. Di conseguenza per rappresentare gli ostacoli posso sfruttare una
% meshgrid. 

%% Grid
nr = 100; nc = 100;
grid = zeros(nr,nc);
[X,Y] = meshgrid(linspace(0,50,nc),linspace(0,50,nr));

%% Obstacles

%walls
grid(1:2,1:100) = 1;         obstacles(1,:) = [X(1,1) X(1,100) Y(1) Y(2)];
grid(99:100,1:100) = 1;    obstacles(2,:) = [X(1,1) X(1,100) Y(99) Y(100)];
grid(1:100,1:2) = 1;         obstacles(3,:) = [X(1,1) X(1,2) Y(1) Y(100)];
grid(1:100,99:100) = 1;    obstacles(4,:) = [X(1,99) X(1,100) Y(1) Y(100)];

%obstacles
grid(15:35,60:75) = 1;      obstacles(5,:) = [X(1,60) X(1,75) Y(15) Y(35)];
grid(15:35,19:20) = 1;      obstacles(6,:) = [X(1,19) X(1,20) Y(15) Y(35)];
grid(15:35,29:30) = 1;      obstacles(7,:) = [X(1,29) X(1,30) Y(15) Y(35)];
grid(49:50,15:70) = 1;      obstacles(8,:) = [X(1,15) X(1,70) Y(49) Y(50)];
grid(60:75,70:85) = 1;      obstacles(9,:) = [X(1,70) X(1,85) Y(60) Y(75)];
grid(70:90,20:40) = 1;      obstacles(10,:) = [X(1,20) X(1,40) Y(70) Y(90)];


%% Plot
Z = 1.*grid; 
figure(1),axis("equal"), axis([0 50 0 50]), hold on; 
s = mesh(X,Y,Z);

figure(2),axis("equal"), axis([0 50 0 50]), hold on; 
for i = 1 : 10
    temp = obstacles(i,:);
    rectangle("position",[temp(1) temp(3) temp(2)-temp(1) temp(4)-temp(3)],"facecolor","r");
end
