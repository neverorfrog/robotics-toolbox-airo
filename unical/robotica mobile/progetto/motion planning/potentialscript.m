function traj = potentialscript(pstart,pgoal)

%% Generazione ambiente con gli ostacoli
[X,Y,grid,obstacles] = ostacoli();

%% Potenziale repulsivo

% L'elemento (i,j) di d contiene la minima distanza dell'elemento (i,j) della
% griglia binaria rispetto ai punti di grid che sono settati a 1, ovvero quelli in cui
% si trovano gli ostacoli
d = bwdist(grid); vd = d(:);
soglia = 2; %soglia di avvicinamento all'ostacolo oltre il quale il potenzial repulsivo aumenta

%Genero la una matrice della stessa dimensione della meshgrid: ogni cella contiene la
%valutazione del potenziale repulsivo in quel punto. Il valore sará zero se sono
%oltre la soglia di avvicinamento, mentre tenderá a infinito (qui lo faccio tendere
%al massimo del potenziale totale) man mano che mi avvicino all'ostacolo.
frep = 1/2*((1./d - 1/soglia).^2); 
frep(d > soglia) = 0;


%% Potenziale attrattivo

%Potenziale attrattivo a forma di paraboloide, con punto di minimo in pgoal
fatt = 1/2*( (X - pgoal(1)).^2 + (Y - pgoal(2)).^2 );


%% Potenziale totale

% Converto i valori infiniti, non contemplabili in situazioni reali, in maxi
maxi = max(fatt(~isinf(fatt))); frep(isinf(frep)) = maxi;
%Sommo i due potenziali
ftot = fatt + frep;

%% Antigradiente e plotting

[gx, gy] = gradient(-frep,50/100); 
% Una versione naive dei potenziali vorticosi, che consiste semplicemente nel ruotare
% di -pi/2 l'antigradiente del potenziale repulsivo, che quindi non punterá piú via
% dall'ostacolo, ma sempre in direzione oraria rispetto all'ostacolo. Il potenziale
% vorticoso entra in gioco lí dove c'é il rischio di un deadlock della ricerca del
% cammino, che potrebbe fermarsi in un punto di minimo locale, oppure non arrestarsi
% mai.
gx1 = gy; gy1 = -gx;

[gx2, gy2] = gradient(-fatt,50/100); 

gxtot = gx1+gx2; gytot = gy1+gy2;

% Chiaramente all'interno dell'ostacolo pongo a zero il gradiente, da non rischiare
% di attraversarlo per sbaglio (pesando male k ad esempio)
for i = 1 : size(grid,1)
    for j = 1 : size(grid,2)
        if grid(i,j) == 1
            gxtot(i,j) = 0; gytot(i,j) = 0;
        end
    end
end


figure(2); quiver(X(1,:), Y(:,1), gxtot, gytot,"b");
figure(3); m = mesh(ftot); hold on;

%% Generazione del percorso

% Devo ora seguire il percorso dettato dall'antigradiente del potenziale totale, che
% mi porterá a raggiungere il punto di minimo. A tal fine sfrutto semplicemente una
% ricerca iterativa, scalata da un indice k (che diventa piú piccolo se mi avvicino
% all'ostacolo).  

traj = [pstart]; e = norm(pstart-pgoal); i = 0; k = 0.01;
j = 1;
while e > 0.3
    i = i+1;
    x0 = traj(i,1); y0 = traj(i,2);
%     Cerco il prossimo punto della meshgrid piú vicino al punto su cui mi ha mandato
%     l'antigradiente e quindi sfrutto questo come punto di partenza della prossima
%     iterazione
    t = (abs(X - x0) < 0.5) & (abs(Y - y0) < 0.5);
    index = find(t); 
%     Check di robustezza 
    if ~isempty(index)
        index = index(randperm(length(index),1));
    else
        k = 0.001;
    end
    fgrad = [gxtot(index) gytot(index)];
    if vd(index) <= 4
        k = 0.001;
    end
%     Aggiorno quindi l'algoritmo in vista della nuova iterazione
    x1 = x0 + k*fgrad(1); 
    y1 = y0 + k*fgrad(2);
    e = norm([x1,y1]  - pgoal);
    traj = [ traj ; x1 y1];
%     Per evitare deadlock, se sono oltre i 10000 cicli, interrompo
    j = j + 1;
    if j > 10000
        break;
    end
end

%% Ottimizzazione del percorso

% Per alleggerire la traiettoria

for i = 2 : length(traj)
    traj(i,3) = atan2(traj(i,2)-traj(i-1,2),traj(i,1)-traj(i-1,1)); 
end

for i = length(traj) : -1 : 2
%     theta uguali
    while i <= length(traj) && abs(traj(i,3)-traj(i-1,3)) < 0.1 && i > 1
        traj(i-1,:) = [];
    end
end

for i = length(traj) : -1 : 2
    %distanza molto piccola tra un punto e un altro
    while i <= length(traj) && norm(traj(i,1:2)-traj(i-1,1:2)) < 0.1 && i > 1
        traj(i-1,:) = [];
    end
end

traj = traj(:,1:2);
