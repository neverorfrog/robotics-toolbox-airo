close all; clear;

%% Trajectory planning

%Points to be visited
P1 = [0 0 0.5]; P2 = [1 0 0.5]; P3 = [0.5 0 1.5]; P4 = [1 1 1];
P = [P1 ; P2 ; P3 ; P1];
T = linspace(0,10,4)';
tsim = []; ttotal = [];
for i = 1 : length(T)-1
    temp = linspace(T(i),T(i+1),50)';
    tsim = [tsim , temp];
    ttotal = [ttotal ; temp];
end


%% Spline

syms("a",[length(P)-1,4,3]);
syms t;

for k = 1 : 3
    %Polinomi
    for i = 1 : length(P)-1
        poli(i,1,k) = a(i,4,k)*t^3+a(i,3,k)*t^2+a(i,2,k)*t+a(i,1,k);
    end
    polidot = diff(poli);
    poliddot = diff(polidot);
end

%Condizioni per trovare i coefficienti

for k = 1 : 3
    
    %Derivate iniziale e finale
    cond(1,1,k) = ...
        subs(polidot(1,1,k),t,T(1)) == 0;
    cond(4*(length(P)-1),1,k) = ...
        subs(polidot(length(polidot),1,k),t,T(length(T))) == 0;
    
    %Posizione iniziale e finale
    cond(2,1,k) = ...
        subs(poli(1,1,k),t,T(1)) == P(1,k);
    cond(4*(length(P)-1)-1,1,k) = ...
        subs(poli(length(poli),1,k),t,T(length(T))) == P(length(P),k);
     
        
    %Posizioni
    j = 1;
    for i = 3 : 4 : 4*(length(P)-1)-2
        
        j = j+1;
         
        cond(i,1,k) = ...
            subs(poli(j-1,1,k),t,T(j)) == P(j,k);
        
        cond(i+1,1,k) = ...
            subs(poli(j,1,k),t,T(j)) == P(j,k);
        
        cond(i+2,1,k) = ...
            subs(polidot(j-1,1,k),t,T(j)) == subs(polidot(j,1,k),t,T(j));
        
        cond(i+3,1,k) = ...
            subs(poliddot(j-1,1,k),t,T(j)) == subs(poliddot(j,1,k),t,T(j));
    end
      
end

coeffxcell = struct2cell(vpasolve(cond(:,:,1)));
coeffycell = struct2cell(vpasolve(cond(:,:,2)));
coeffzcell = struct2cell(vpasolve(cond(:,:,3)));

for i = 4 : 4 : length(coeffxcell)
    
    coeffx(i/4,4) = double(coeffxcell{i-3});
    coeffy(i/4,4) = double(coeffycell{i-3});
    coeffz(i/4,4) = double(coeffzcell{i-3});
    
    coeffx(i/4,3) = double(coeffxcell{i-2});
    coeffy(i/4,3) = double(coeffycell{i-2});
    coeffz(i/4,3) = double(coeffzcell{i-2});
    
    coeffx(i/4,2) = double(coeffxcell{i-1});
    coeffy(i/4,2) = double(coeffycell{i-1});
    coeffz(i/4,2) = double(coeffzcell{i-1});
   
    coeffx(i/4,1) = double(coeffxcell{i});
    coeffy(i/4,1) = double(coeffycell{i});
    coeffz(i/4,1) = double(coeffzcell{i});

end


X = []; Y = []; Z = [];

for i = 1 : size(coeffx,1)
    X = [X;polyval(coeffx(i,:),tsim(:,i))];
    Y = [Y;polyval(coeffy(i,:),tsim(:,i))];
    Z = [Z;polyval(coeffz(i,:),tsim(:,i))];
end

%% Plotting

plot3(X,Y,Z), hold on, grid on;
xlabel('X'); ylabel('Y'); zlabel("Z");
plot3(P(:,1),P(:,2),P(:,3),'o')
title("curva continua con 4 nodi");

%Grafico x-t, y-t, z-t
figure(2); grid; hold on;
subplot(3,1,1);plot(ttotal,X); title("X");
subplot(3,1,2);plot(ttotal,Y); title("Y");
subplot(3,1,3);plot(ttotal,Z); title("Z");

%% Inversione cinematica
Q = ones(150,3); Qdot = ones(150,3);
for i = 1 : length(X)
    Qtemp = invkin_ant([X(i) Y(i) Z(i)],[0 0.75 0.75]);
    jacobian = jacobian_ant(Qtemp(:,:,2),L);
    Qdottemp = (jacobian \ vel(i,:)')';
    Q(i,:) = Qtemp(:,:,2);
    Qdot(i,:) = Qdottemp;
end

%% Plotting
figure(1);    
%Plot dei bracci del robot al variare della posizione sulla retta
for i = 1 : length(X)
    q1=Q(i,1); q2=Q(i,2); q3=Q(i,3);
    %primo braccio (fisso, perché fa da base rotante)
    P0P1 = [0.5 0.5 0 ; 0.5 0.5 1];
    plot3(X,Y,Z); axis([-1 1.8 -1 1.8 -1 1.8]),hold on,grid on;
    plot3(P0P1(:,1) , P0P1(:,2) , P0P1(:,3)); 
    %secondo braccio
    P1P2 = [P0P1(2,:) ; [0.75*cos(q2)*cos(q1)+0.5 , 0.75*cos(q2)*sin(q1)+0.5 , 0.75*sin(q2)+1]];
    plot3(P1P2(:,1) , P1P2(:,2) , P1P2(:,3));
    %terzo braccio
    P2P3 = [P1P2(2,:) ; ...
        cos(q1)*0.75*(cos(q2) + cos(q3+q2)) + 0.5, sin(q1)*0.75*(cos(q2) + cos(q3+q2)) + 0.5,...
        0.75*(sin(q2) + sin(q2+q3)) + 1];
    plot3(P2P3(:,1) , P2P3(:,2) , P2P3(:,3));
    
    pause(0.01); hold off;
end
