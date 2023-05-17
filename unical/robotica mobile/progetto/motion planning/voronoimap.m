function traj = voronoimap(pstart,pgoal)

%% Generazione ostacoli
[X,Y,grid,obstacles] = ostacoli();

xstart = pstart(1); xgoal = pgoal(1);
ystart = pstart(2); ygoal = pgoal(2);

%% Calcolo mappa di voronoi sui punti della meshgrid
x = X.*grid; y = Y.*grid; x = x(:); y = y(:);
[vx,vy] = voronoi(x,y);

%% Pulitura mappa di voronoi

% Per ogni coppia di vertici in [vx,vy], dovró scorrere tutti gli ostacoli e vedere
% se ci sono delle intersezioni: basta che il segmento tra i due vertici intersechi
% un solo vertice e il segmento viene cancellato dalla mappa.
for i = length(vx) : -1 : 1
    
    xline = vx(:,i); yline = vy(:,i);
    
    for j = 1 : size(obstacles,1)
        xobst = [obstacles(j,1) obstacles(j,2)];
        yobst = [obstacles(j,3) obstacles(j,4)];
  
        if obstacleIntersect2(xline,yline,xobst,yobst)
            vx(:,i) = []; vy(:,i) = [];
            break;
        end
    end
end


%% Ritrazione 

% Unisco i punti di start e goal alla mappa di voronoi generata. In realtá qui la
% ritrazione avviene in maniera molto naive: si dovrebbe controllare se innanzitutto
% esiste e in caso se i due segmenti non intersecano nessun ostacolo. Qui connetto i
% punti di start e goal semplicemente al nodo piú vicino della mappa di voronoi.

% Metto tutti i vertici in unico vettore v
v = [];
for i = 1 : length(vx)
    v = [v ; [vx(1,i) vy(1,i)] ; [vx(2,i) vy(2,i)]];
end

% Cerco quindi tra i vertici di v quello che é meno distante da start e goal e unisco
% quest'ultimi al vettore v
vnormstart = v-[xstart ystart];
vnormgoal = v-[xgoal ygoal];
for i = 1 : size(v,1)
    vnormstart(i,:) = norm(vnormstart(i,:));
    vnormgoal(i,:) = norm(vnormgoal(i,:));
end
[~,istart] = min(vnormstart);
[~,igoal] = min(vnormgoal);
vstart = v(istart(1),:); vgoal = v(igoal(1),:);

vx = [vx  [xstart ; vstart(1)]  [xgoal ; vgoal(1)]];
vy= [vy   [ystart ; vstart(2)]  [ygoal ; vgoal(2)]];
v = [v ; [xstart ystart] ; [vstart(1) vstart(2)] ; [xgoal ygoal] ; [vgoal(1) vgoal(2)]];


%% Plot della mappa finale

figure(2);plot(vx,vy,"g","linewidth",2); hold on;
xstart = pstart(1); ystart = pstart(2); xgoal = pgoal(1); ygoal = pgoal(2);
plot(xstart,ystart,"ok","linewidth",2);
plot(xgoal,ygoal,"ok","linewidth",2);

%% Creazione grafo tramite matrice di adiacenza

% Siccome in v la sequenza di segmenti é alla rinfusa, vertici adiacenti possono 
% essere in posizioni molto diverse. Quindi, tramite findZeroRows trovo vertici 
% coincidenti in v. In seguito, si segue il principio che i vertici viaggiano a coppie 
% in cui il primo vertice é di indice dispari e il secondo di indice pari.

A = zeros(length(v));
vtemp = v;
for i = length(vtemp) : -1 : 1
    temp = (vtemp-vtemp(i,:));
    occ = findZeroRows(temp);
    for j = 1 : length(occ)
        index = occ(j);
        if mod(index,2) == 0
            A(i,index-1) = 1;
        elseif index < length(vtemp)
            A(i,index+1) = 1;
        end
    end
    A(i,i) = 0;
    vtemp(i,:) = [];
end

G = graph(A,"lower");
figure(2); plot(G,"xdata",v(:,1),"ydata",v(:,2));

path = shortestpath(G,length(v)-3,length(v)-1);
traj = v(path,:);

%% Ottimizzazione percorso

for i = 2 : length(traj)
    traj(i,3) = atan2(traj(i,2)-traj(i-1,2),traj(i,1)-traj(i-1,1)); 
end
for i = length(traj) : -1 : 2
    while abs(traj(i,3)-traj(i-1,3)) < 0.01 && i > 1
        traj(i-1,:) = [];
    end
end
