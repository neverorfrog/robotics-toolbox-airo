function traj = discretepotential(pstart,pgoal)

%% Generazione ostacoli
[X,Y,grid,obstacles] = ostacoli();

%% Inizializzazione

cspace = zeros(100,100);
igoalx = floor((pgoal(1)*100)/50);
igoaly = floor((pgoal(2)*100)/50);
istartx = floor((pstart(1)*100)/50);
istarty = floor((pstart(2)*100)/50);


%% Riempimento griglia

for i = 1 : 100
    
    %colonna destra
    l = igoalx + i;
    for k = igoaly - i : igoaly + (i-1)
        if k < 1 || l < 1 || k > 100 || l > 100
        else
            if grid(k,l) == 1
                cspace(k,l) = inf;
            else
                cspace(k,l) = i;
            end
        end
    end
    %colonna sinistra
    l = igoalx - i;
    for k = igoaly + i : -1 : igoaly - (i-1)
        if k < 1 || l < 1 || k > 100 || l > 100
        else
            if grid(k,l) == 1
                cspace(k,l) = inf;
            else
                cspace(k,l) = i;
            end
        end
    end
    %riga sopra
    k = igoaly - i;
    for l = igoalx - i : igoalx + (i-1)
        if k < 1 || l < 1 || k > 100 || l > 100
        else
            if grid(k,l) == 1
                cspace(k,l) = inf;
            else
                cspace(k,l) = i;
            end
        end
    end
    %riga sotto
    k = igoaly + i;
    for l = igoalx + i : -1 : igoalx - (i-1)
        if k < 1 || l < 1 || k > 100 || l > 100
        else
            if grid(k,l) == 1
                cspace(k,l) = inf;
            else
                cspace(k,l) = i;
            end
        end
    end
    
end

Z = cspace.*(grid+1);
figure(3); mesh(X,Y,Z);
%% Traiettoria

c = cspace;
found = false; i = istartx; j = istarty;
traj = [X(1,j) Y(i,1)];
global verso; verso = "orario";

while ~found
    
    %Estraggo l'elemento minimo del mio intorno. Se ce n'é piú di uno prendo il primo
    current = c(i,j);
    temp = c(i-1:i+1,j-1:j+1);
    [elem,index] = min(temp,[],"linear");
    [next,ind] = min(elem);
    [k,l] = ind2sub(3,index(ind));
    c(i,j) = 200;
    itemp = i + (k-2); jtemp = j + (l-2);
    
    %Caso in cui sono sul bordo di un ostacolo. Inizio quindi a "cirumnavigarlo"
    %finché non trovo una casella piú piccola che mi allontani da esso.
    if ~isempty(find(isinf(temp),1))
        [k,l] = circum(isinf(temp));
    end
    i = i + (k-2); j = j + (l-2);
    
    %Check se c'é un elemento nell'intorno che é piú piccolo del prossimo se
    %continuassi a circumnavigare l'ostacolo.
    if next  < c(i,j)
        i = itemp; j = jtemp;
    end
    
    %finisco solo se sono arrivato alla cella con valore zero, che é il goal  
    if current == 0
        found = 1;
    end
    
    %Aggiorno la traiettoria
    traj = [traj ; X(1,j) Y(i,1)];
end

%% Ottimizzazione percorso

% Per evitare di interpolare tra di loro punti che si trovano su un segmento
% rettlineo e creare cosí un andamento singhiozzante nel plot

for i = 2 : length(traj)
    traj(i,3) = atan2(traj(i,2)-traj(i-1,2),traj(i,1)-traj(i-1,1)); 
end
for i = length(traj) : -1 : 2
    while abs(traj(i,3)-traj(i-1,3)) <= 0.001 && i > 1
        traj(i-1,:) = [];
    end
end
traj = traj(:,1:2);
