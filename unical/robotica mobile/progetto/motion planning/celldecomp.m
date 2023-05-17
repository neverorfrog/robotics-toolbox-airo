function traj = celldecomp(pstart,pgoal)

%% Generazione ambiente con gli ostacoli

[X,Y,grid,obstacles] = ostacoli(); hold on;

edgex = []; edgey = [];


%% Decomposizione in celle

% Praticamente tiro a ogni vertice una linea verso sotto e verso sopra. Raccolgo poi
% tutti i punti di intersezione, per poi selezionarne il sup dei punti con ordinata
% piú piccola del vertice e l'inf dei punti con ordinata piú grande. Saranno i due
% limiti del segmento
for i = 1 : size(obstacles,1)
    
    yi = obstacles(i,3); yf = obstacles(i,4);
    x = [obstacles(i,1) , obstacles(i,2)];
    
    for k = 1 : 2
        if x(k) < 49 && x(k) > 1
            y = [];
            
            for j = 1 : size(obstacles,1)
                if i ~= j
                    yobst = [obstacles(j,3) obstacles(j,4)];
                    xobst = [obstacles(j,1) obstacles(j,2)];
                    if xobst(1) > x(k) || xobst(2) < x(k)
                    else
                        y = [y ; yobst(1) ; yobst(2)];
                    end
                end
            end
            
            yinf = []; ysup = [];
            for j = 1 : length(y)
                if y(j) < yi
                    yinf = [yinf ; y(j)];
                elseif y(j) > yf
                    ysup = [ysup ; y(j)];
                end
            end
            if ~isempty(ysup)
                edgey = [edgey ; [yf min(ysup)]];
                edgex = [edgex ; [x(k) x(k)]];
            end
            if ~isempty(yinf)
                edgey = [edgey ; [yi max(yinf)]];
                edgex = [edgex ; [x(k) x(k)]];
            end
        end
    end
end

% Riempio quindi un vettore con i punti dei segmenti trovati e li collego
conjpoints = [];
for i = 1 : size(edgex,1)
    mid = (edgey(i,1)+edgey(i,2))/2;
    conjpoints = [conjpoints ; [edgex(i,1) mid]];
    plot(edgex(i,:),edgey(i,:),"g","linewidth",2);
    plot(edgex(i,:),mid,"ok","linewidth",2);
end
conjpoints = unique(double(conjpoints),"rows");
edges = [edgex,edgey];

%% Grafo


% Devo pulire il grafo dei segmenti che intersecano ostacoli o altri segmenti, in
% modo da collegare soltanto celle contigue tra di loro
A = zeros(length(conjpoints));

for i = 1 : length(conjpoints)-1
    for j = i+1 : length(conjpoints)
        xline = [conjpoints(i,1) conjpoints(j,1)];
        yline = [conjpoints(i,2) conjpoints(j,2)];
        r = 0;
        
        for k = 1 : size(obstacles,1)
            xobst = [obstacles(k,1) obstacles(k,2)];
            yobst = [obstacles(k,3) obstacles(k,4)];
            if obstacleIntersect2(xline,yline,xobst,yobst)
                r = 1; break;
            end
        end
        
        %entro solo se hanno ascissa diversa
        if conjpoints(i,1) ~= conjpoints(j,1)
            for k = 1 : size(edges,1)
                xline2 = [edges(k,1) edges(k,2)];
                yline2 = [edges(k,3) edges(k,4)];
                if lineIntersect(xline,yline,xline2,yline2)
                    r = 1; break;
                end
            end
        end
        
        if r == 0
            A(i,j) = 1;
        end
        
    end
end


% Cerco quindi tra i vertici di v quello che é meno distante da start e goal e unisco
% quest'ultimi al vettore v
v = conjpoints;
xstart = pstart(1); xgoal = pgoal(1); ystart = pstart(2); ygoal = pgoal(2);
vnormstart = v-[xstart ystart];
vnormgoal = v-[xgoal ygoal];
for i = 1 : size(v,1)
    vnormstart(i,:) = norm(vnormstart(i,:));
    vnormgoal(i,:) = norm(vnormgoal(i,:));
end
[~,istart] = min(vnormstart);
[~,igoal] = min(vnormgoal);
v = [v ; [xstart ystart] ;[xgoal ygoal]];
conjpoints = v;


G = graph(A,"upper");
G = addedge(G,length(A)+1,istart(1),1);G = addedge(G,length(A)+2,igoal(1),1);
figure(2); plot(G,"b","linewidth",2,"xdata",conjpoints(:,1),"ydata",conjpoints(:,2));
figure(3); plot(G,"b","linewidth",2,"xdata",conjpoints(:,1),"ydata",conjpoints(:,2));

path = shortestpath(G,length(A)+1,length(A)+2);
traj = conjpoints(path,:);
