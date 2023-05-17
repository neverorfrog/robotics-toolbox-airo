% Funzione che, dato in ingresso un vettore v, restituisce gli indici di tutte le
% righe che si trovano interamente sotto una certa soglia

function index = findZeroRows(v)

index = [];

for i = 1 : size(v,1)
    check = 1;
    for j = 1 : size(v,2)
        if abs(v(i,j)) > 0.001
            check = 0; break;
        end
    end
    if check == 1
        index = [index ; i];
    end
end
