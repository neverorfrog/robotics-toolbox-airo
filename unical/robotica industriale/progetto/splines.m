close all; clear;

%% Trajectory planning

%Points to be visited
Pb = [0 0 0.5 ; 1 0 0.5 ; 0.5 0 1.5 ; 0 0 0.5];
Rb0 = [1 0 0 0.5 ; 0 1 0 0.5 ; 0 0 1 1 ; 0 0 0 1];
P1 = Rb0 \ [0 0 0.5 1]'; P2 = Rb0 \ [1 0 0.5 1]'; P3 = Rb0 \ [0.5 0 1.5 1]';
P = [P1(1:3)' ; P2(1:3)' ; P3(1:3)' ; P1(1:3)'];
T = linspace(0,10,4)';
tsim = []; ttotal = [];
for i = 1 : length(T)-1
    temp = linspace(T(i),T(i+1),50)';
    tsim = [tsim , temp];
    ttotal = [ttotal ; temp];
end

%% Matrice di Vandermonde

% V = vander(T);
% px = P(:,1); py = P(:,2); pz = P(:,3);
% A = V \ px; B = V \ py; C = V \ pz;
% X = polyval(A,ttotal); Y = polyval(B,ttotal); Z = polyval(C,ttotal);

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
    cond(1,1,k) = subs(polidot(1,1,k),t,T(1)) == 0;
    cond(4*(length(P)-1),1,k) = subs(polidot(length(polidot),1,k),t,T(length(T))) == 0;    
    %Posizione iniziale e finale
    cond(2,1,k) = subs(poli(1,1,k),t,T(1)) == P(1,k);
    cond(4*(length(P)-1)-1,1,k) = subs(poli(length(poli),1,k),t,T(length(T))) == P(length(P),k);
    
    j = 1;
    for i = 3 : 4 : 4*(length(P)-1)-2
        
        j = j+1;
        
        %Posizioni
        cond(i,1,k) = subs(poli(j-1,1,k),t,T(j)) == P(j,k);
        cond(i+1,1,k) = subs(poli(j,1,k),t,T(j)) == P(j,k);
        %Derivate
        cond(i+2,1,k) = subs(polidot(j-1,1,k),t,T(j)) == subs(polidot(j,1,k),t,T(j));
        cond(i+3,1,k) = subs(poliddot(j-1,1,k),t,T(j)) == subs(poliddot(j,1,k),t,T(j));
    end
    
end

%Risolvo il sistema e genero la traiettoria con le spline
coeff = vpasolve(cond);
poli = subs(poli,coeff); polidot = subs(polidot,coeff);
X = []; Y = []; Z = []; Xdot = []; Ydot = []; Zdot = [];


%Questa traiettoria naturalmente é quella vista dal sistema SR0. Per ottenere la
%traiettoria nel sistema SRb dobbiamo riconvertire i punti.
for i = 1 : 3
    X = [X;polyval(double(coeffs(poli(i,:,1),"all")),tsim(:,i))];
    Y = [Y;polyval(double(coeffs(poli(i,:,2),"all")),tsim(:,i))];
    Z = [Z;polyval(double(coeffs(poli(i,:,3),"all")),tsim(:,i))];
    
    Xdot = [Xdot ; polyval(double(coeffs(polidot(i,:,1),"all")),tsim(:,i))];
    Ydot = [Ydot ; polyval(double(coeffs(polidot(i,:,2),"all")),tsim(:,i))];
    Zdot = [Zdot ; polyval(double(coeffs(polidot(i,:,3),"all")),tsim(:,i))];
end



% Inversione cinematica offline del robot antropomorfo
Q = ones(150,3); Qdot = ones(150,3); syms q1 q2 q3;
L = [0 0.75 0.75];
for i = 1 : length(X)
    Qtemp = invkin_ant([X(i) Y(i) Z(i)],[0 0.75 0.75]);
    jacobian = jacobian_ant(Qtemp(:,:,2),dirkin_ant([q1 q2 q3],L),L);
    Qdottemp = (jacobian \ [Xdot(i) ; Ydot(i) ; Zdot(i)])';
    Q(i,:) = Qtemp(:,:,2);
    Qdot(i,:) = Qdottemp;
end


%% Plotting

figure(1);
xlabel('X'); ylabel('Y'); zlabel("Z");
title("curva continua con 4 nodi");

%Plot dei bracci del robot al variare della posizione sulla retta
for i = 1 : length(X)
    
    plot3(Pb(1,1),Pb(1,2),Pb(1,3),"or","linewidth",2); hold on; grid on;
    plot3(Pb(2,1),Pb(2,2),Pb(2,3),"or","linewidth",2);
    plot3(Pb(3,1),Pb(3,2),Pb(3,3),"or","linewidth",2);
    axis([-1 1.8 -1 1.8 -1 1.8]),hold on;
    
    q1=Q(i,1); q2=Q(i,2); q3=Q(i,3);
    
    R03 = dirkin_ant(Q(i,:),[0 0.75 0.75]);
    Rb3 = Rb0 * R03; %rototraslazioni in terna mobile
    X(i) = Rb3(1,4); Y(i) = Rb3(2,4); Z(i) = Rb3(3,4);
    %primo braccio (fisso, perché fa da base rotante)
    P0P1 = [0.5 0.5 0 ; 0.5 0.5 1];
    b1 = plot3(P0P1(:,1) , P0P1(:,2) , P0P1(:,3),"g","linewidth",2); 
    
    %secondo braccio
    Rtemp = [1 0 0 -0.75*cos(q1)*cos(q2+q3) ; 0 1 0 -0.75*sin(q1)*cos(q2+q3) ; 0 0 1 -0.75*sin(q2+q3) ; 0 0 0 1];
    Rb2 = Rtemp*Rb3; %rototraslazioni in terna fissa b
    P1P2 = [ P0P1(2,:) ; Rb2(1:3,4)'];
    b2 = plot3(P1P2(:,1) , P1P2(:,2) , P1P2(:,3),"g","linewidth",2);
    
    %terzo braccio
    P2P3 = [P1P2(2,:) ; Rb3(1:3,4)'];
    b3 = plot3(P2P3(:,1) , P2P3(:,2) , P2P3(:,3),"g","linewidth",2);
    
    plot3(X(i),Y(i),Z(i),"ob");
    pause(0.1);
    delete(b1); delete(b2); delete(b3);
end

Q=Q*180/pi; %Le esprimo in gradi in centigradi
Qdot = Qdot*180/pi;

%Grafico x-t, y-t, z-t
figure(2);
subplot(3,1,1);plot(ttotal,X(:),"b",T(1),X(1),"or",T(2),X(50),"or",T(3),X(100),"or"); title("X"); grid;
subplot(3,1,2);axis([0 10 0 1]), hold on; plot(ttotal,Y(:),"b",T(1),Y(1),"or",T(2),Y(50),"or",T(3),Y(100),"or"); title("Y"); grid;
subplot(3,1,3);plot(ttotal,Z(:),"b",T(1),Z(1),"or",T(2),Z(50),"or",T(3),Z(100),"or"); title("Z"); grid;

%Grafico xdot-t, ydot-t, zdot-t
figure(3);
subplot(3,1,1);plot(ttotal,Xdot,"b",T(1),Xdot(1),"or",T(2),Xdot(50),"or",T(3),Xdot(100),"or"); title("Xdot"); grid;
subplot(3,1,2);axis([0 10 0 1]), hold on; plot(ttotal,Ydot,"b",T(1),Ydot(1),"or",T(2),Ydot(50),"or",T(3),Ydot(100),"or"); title("Ydot"); grid;
subplot(3,1,3);plot(ttotal,Zdot,"b",T(1),Zdot(1),"or",T(2),Zdot(50),"or",T(3),Zdot(100),"or"); title("Zdot"); grid;

%Grafico q1-t, q2-t, q3-t
figure(4); 
subplot(3,1,1);plot(ttotal,Q(:,1),"b",T(1),Q(1,1),"or",T(2),Q(50,1),"or",T(3),Q(100,1),"or"); title("q1"); grid;
subplot(3,1,2);plot(ttotal,Q(:,2),"b",T(1),Q(1,2),"or",T(2),Q(50,2),"or",T(3),Q(100,2),"or"); title("q2"); grid;
subplot(3,1,3);plot(ttotal,Q(:,3),"b",T(1),Q(1,3),"or",T(2),Q(50,3),"or",T(3),Q(100,3),"or"); title("q3"); grid;

%Grafico q1dot-t, q2dot-t, q3dot-t
figure(5);
subplot(3,1,1);plot(ttotal,Qdot(:,1),"b",T(1),Qdot(1,1),"or",T(2),Qdot(50,1),"or",T(3),Qdot(100,1),"or"); title("q1dot"); grid;
subplot(3,1,2);plot(ttotal,Qdot(:,2),"b",T(1),Qdot(1,2),"or",T(2),Qdot(50,2),"or",T(3),Qdot(100,2),"or"); title("q2dot"); grid;
subplot(3,1,3);plot(ttotal,Qdot(:,3),"b",T(1),Qdot(1,3),"or",T(2),Qdot(50,3),"or",T(3),Qdot(100,3),"or"); title("q3dot"); grid;

