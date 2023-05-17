close all; clear all;

%% Punti della traiettoria

Pb = [0 0 0.5 ; 1 0 0.5 ; 0.5 0 1.5 ; 0 0 0.5];
Rb0 = [1 0 0 0.5 ; 0 1 0 0.5 ; 0 0 1 1 ; 0 0 0 1];
P1 = Rb0 \ [0 0 0.5 1]'; P2 = Rb0 \ [1 0 0.5 1]'; P3 = Rb0 \ [0.5 0 1.5 1]';
P = [P1(1:3)' ; P2(1:3)' ; P3(1:3)' ; P1(1:3)'];

L = [0 0.75 0.75];
T = linspace(0,10,4)';
tsim = []; ttotal = [];
for i = 1 : length(T)-1
    temp = linspace(T(i),T(i+1),50)';
    tsim = [tsim , temp];
    ttotal = [ttotal ; temp];
end

%% Profilo velocitá polinomiale

syms s a0 a1 a2 a3 a4 a5;
lambdas = a0*s^5 + a1*s^4 + a2*s^3 + a3*s^2 +a4*s + a5;
lambdadots = diff(lambdas); lambdaddots = diff(lambdadots);
%conditions
c1 = subs(lambdas,s,0) == 0; c4 = subs(lambdas,s,1) == 1;
c2 = subs(lambdadots,s,0) == 0; c5 = subs(lambdadots,s,1) == 0;
c3 = subs(lambdaddots,s,0) == 0; c6 = subs(lambdaddots,s,1) == 0; 
%system solver
[a0 a1 a2 a3 a4 a5] = vpasolve([c1 c2 c3 c4 c5 c6]);

lambda = subs(lambdas);
lambdadot = diff(lambda);
lambdaddot = diff(lambdadot);

pos = []; vel = []; lamb = []; lambdot = [];
for i=1 : size(tsim,2)
    for j = 1 : size(tsim,1)
        stemp = (tsim(j,i)-T(i))/(T(i+1)-T(i));
        stempdot = 1/(T(i+1)-T(i));
        lamb = [lamb ; subs(lambda,s,stemp)];
        lambdot = [lambdot ; subs(lambdadot,s,stemp)];
        pos = [ pos ; P(i,:) + lamb(end)*(P(i+1,:)-P(i,:)) ];
        vel = [ vel ; lambdot(end)*(P(i+1,:)-P(i,:))*stempdot ];
    end
end
pos = double(pos); vel = double(vel);

%% Inversione cinematica

syms q1 q2 q3;
Q = ones(150,3); Qdot = ones(150,3);
R = dirkin_ant([q1 q2 q3],L);

for i = 1 : length(pos)
    
    Qtemp = invkin_ant(double(pos(i,:)),L);
    jacobian = jacobian_ant(Qtemp(:,:,2),R,L);
    Qdottemp = (jacobian \ vel(i,:)')';
    Q(i,:) = Qtemp(:,:,2);
    Qdot(i,:) = Qdottemp;
    
end

%% Plotting

%Plot "dinamico" del percorso nello spazio tridimensionale
figure(1);
xlabel("X"); ylabel("Y"); zlabel("Z");


Rb0 = [1 0 0 0.5 ; 0 1 0 0.5 ; 0 0 1 1 ; 0 0 0 1];

for i=1 : length(Q)
    
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

%Grafico x-t, y-t, z-t
figure(2);
subplot(3,1,1);plot(ttotal,X(:),"b",T(1),X(1),"or",T(2),X(50),"or",T(3),X(100),"or"); title("X"); grid;
subplot(3,1,2);axis([0 10 0 1]), hold on; plot(ttotal,Y(:),"b",T(1),Y(1),"or",T(2),Y(50),"or",T(3),Y(100),"or"); title("Y"); grid;
subplot(3,1,3);plot(ttotal,Z(:),"b",T(1),Z(1),"or",T(2),Z(50),"or",T(3),Z(100),"or"); title("Z"); grid;

%Grafico xdot-t, ydot-t, zdot-t
figure(3);
subplot(3,1,1);plot(ttotal,vel(:,1),"b",T(1),vel(1,1),"or",T(2),vel(50,1),"or",T(3),vel(100,1),"or"); title("Xdot"); grid;
subplot(3,1,2);axis([0 10 0 1]), hold on; plot(ttotal,vel(:,2),"b",T(1),vel(1,2),"or",T(2),vel(50,2),"or",T(3),vel(100,2),"or"); title("Ydot"); grid;
subplot(3,1,3);plot(ttotal,vel(:,3),"b",T(1),vel(1,3),"or",T(2),vel(50,3),"or",T(3),vel(100,3),"or"); title("Zdot"); grid;


Q = Q*180/pi; Qdot = Qdot*180/pi;%Le esprimo in gradi in centigradi

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

 
%Grafico x-lambda, y-lambda, z-lambda
figure(6); hold on;
for i = 0 : 50 :100
    subplot(3,1,1);
    plot(lamb(i+1:i+50,1)+i/50,pos(i+1:i+50,1),"+r"),grid; hold on; title("X");
    subplot(3,1,2);
    plot(lamb(i+1:i+50,1)+i/50,pos(i+1:i+50,2),"+g"),grid; hold on; title("Y");
    subplot(3,1,3);
    plot(lamb(i+1:i+50,1)+i/50,pos(i+1:i+50,3),"+k"),grid; hold on; title("Z");
end
xlabel("\lambda");

%Grafico xdot-lambda, ydot-lambda, zdot-lambda
figure(7); hold on;
for i = 0 : 50 :100
    subplot(3,1,1);
    plot(lamb(i+1:i+50,1)+i/50,vel(i+1:i+50,1),"+r"),grid; hold on; title("Xdot");
    subplot(3,1,2);
    plot(lamb(i+1:i+50,1)+i/50,vel(i+1:i+50,2),"+g"),grid; hold on; title("Ydot");
    subplot(3,1,3);
    plot(lamb(i+1:i+50,1)+i/50,vel(i+1:i+50,3),"+k"),grid; hold on; title("Zdot");
end
xlabel("\lambda");
 
%Grafico q1-lambda, q2-lambda, q3-lambda
figure(8); hold on;
for i = 0 : 50 :100
    subplot(3,1,1);
    plot(lamb(i+1:i+50,1)+i/50,Q(i+1:i+50,1),"+r"),grid; hold on; title("q1");
    subplot(3,1,2);
    plot(lamb(i+1:i+50,1)+i/50,Q(i+1:i+50,2),"+g"),grid; hold on; title("q2");
    subplot(3,1,3);
    plot(lamb(i+1:i+50,1)+i/50,Q(i+1:i+50,3),"+k"),grid; hold on; title("q3");
end
xlabel("\lambda");

%Grafico q1dot-lambda, q2dot-lambda, q3dot-lambda
figure(9); hold on;
for i = 0 : 50 :100
    subplot(3,1,1);
    plot(lamb(i+1:i+50,1)+i/50,Qdot(i+1:i+50,1),"+r"),grid; hold on; title("q1dot");
    subplot(3,1,2);
    plot(lamb(i+1:i+50,1)+i/50,Qdot(i+1:i+50,2),"+g"),grid; hold on; title("q2dot");
    subplot(3,1,3);
    plot(lamb(i+1:i+50,1)+i/50,Qdot(i+1:i+50,3),"+k"),grid; hold on; title("q3dot");
end
xlabel("\lambda");
