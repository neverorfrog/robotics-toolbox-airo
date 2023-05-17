%Questa funzione estrae il vettore spostamento in due dimensioni
%P da una matrice di rototraslazione

function P = translation(T)
if T(3,4) == 0
    P = T(1:2,4);
else 
    P = T(1:3,4);
end
