%Ottengo una matrice di rototraslazione tra due terne a partire da una
%generica tabella di Denavit-Hartenberg
%DH = [aI, alphaI, dI, thetaI]. Questa riga viene
%ripetuta per un numero uguale a quello dei giunti

function [T,q] = dirkin(DH)

%Estraggo il numero di giunti dalla tabella, prendendo le righe
nJoints = size(DH,1);
if ( nJoints < 1 )
     fprintf("Parametro invalido, non ci possono essere 0 giunti");
end

%Controllo che il numero di colonne sia 4
if ( size(DH,2) < 4 )
   fprintf("Parametro invalido, le colonne devono essere 4");
end

T = eye(4);
syms("q",[1,nJoints]);

%Scorro le righe per definire le variabili di giunto
for k = 1 : size(DH,1)
    
    %Assegno la variabile all'attuale giunto a seconda della sua tipologia
    if symType(DH(k,4)) == "variable"
       q(k) = DH(k,4); %giunto rotoidale
    elseif symType(DH(k,3)) == "variable"
       q(k) = DH(k,3); %giunto prismatico
    end
    
    %traslazione lungo zi-1 di una distanza di (variabile se prismatico)
    traslZ = [0 0 DH(k,3)]'; 
    %rototraslazione attorno all'asse zi-1 per allineare gli assi x (var se rot)
    hom1 = homZ(DH(k,4),traslZ); 
    
    %traslazione lungo xi di una distanza ai
    traslX = [DH(k,1) 0 0]'; 
    %rototraslazione attorno all'asse xi per allineare gli assi z
    hom2 = homX(DH(k,2),traslX); 
    
    hom = hom1*hom2; %Compongo le matrici di rototraslazione
    
    T = T*hom; %Compongo questa rototraslazione alla precedente
end

% T = simplify(T);

end
