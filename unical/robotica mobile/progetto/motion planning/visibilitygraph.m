function traj = visibilitygraph(pstart,pgoal)

%% Costruzione ambiente con gli ostacoli
[X,Y,grid,obstacles] = ostacoli();
xstart = pstart(1); xgoal = pgoal(1);
ystart = pstart(2); ygoal = pgoal(2);


%% Collegamenti tra vertici visibili

% Estraggo le informazioni sui vertici degli ostacoli, per poi metterli in unico
% vettore v
for i = 5 : size(obstacles,1)
    v(1,:,i-4) = [obstacles(i,1) obstacles(i,3)];
    v(2,:,i-4) = [obstacles(i,1) obstacles(i,4)];
    v(3,:,i-4) = [obstacles(i,2) obstacles(i,3)];
    v(4,:,i-4) = [obstacles(i,2) obstacles(i,4)];
end

% Connetto quindi tutti i vertici tra di loro
linex = []; liney = [];
for i = 1 : size(v,3)
    for j = 1 : 4
        for l = i : size(v,3)
            for k = 1 : 4
                temp = [v(j,1,i) v(k,1,l)];
                linex = [linex ; [temp(1) temp(2)]];
                temp = [v(j,2,i) v(k,2,l)];
                liney = [liney ; [temp(1) temp(2)]];
            end
        end
    end
end

% Plot di tutti collegamenti tra i vertici
% for i = 1 : length(linex)
%     plot(linex(i,:),liney(i,:),"g");
% end

% Controllo per ogni coppia di vertici se essi sono visibili, ovvero che non
% intersechino alcun ostacolo.
for i = length(linex) : -1 : 1
    for j = 1 : size(obstacles,1)
        c = obstacleIntersect2(linex(i,:),liney(i,:),obstacles(j,1:2),obstacles(j,3:4));
        if c | ~(linex(i,:) < 50) | ~(liney(i,:) < 50) | ~(linex(i,:) > 0)
            linex(i,:) = []; liney(i,:) = []; break;
        end
    end
end


% Plot finale dei collegamenti
v = [];
for i = 1 : length(linex)
    plot(linex(i,:),liney(i,:),"g","linewidth",2);
    v = [v ; linex(i,1) liney(i,1) ; linex(i,2) liney(i,2)];
end

% Aggiunta dei punti di start e goal alla mappa dei collegamenti
vnormstart = v-[xstart ystart];
vnormgoal = v-[xgoal ygoal];
for i = 1 : size(v,1)
    vnormstart(i,:) = norm(vnormstart(i,:));
    vnormgoal(i,:) = norm(vnormgoal(i,:));
end
[~,istart] = min(vnormstart);
[~,igoal] = min(vnormgoal);
vstart = v(istart(1),:); vgoal = v(igoal(1),:);
v = [v ; [xstart ystart] ; [vstart(1) vstart(2)] ; [xgoal ygoal] ; [vgoal(1) vgoal(2)]];

%% Creazione grafo di visibilitá

% Sfrutto qui la stessa identica tecnica come in voronoimap
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
    vtemp(i,:) = [];
end

for i = 1 : length(A)
    A(i,i) = 0;
end

G = graph(A,"lower");
figure(2); plot(G,"xdata",v(:,1),"ydata",v(:,2));

path = shortestpath(G,287,289);
traj = v(path,:);
