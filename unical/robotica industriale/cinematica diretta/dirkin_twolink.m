function T = dirkin_twolink(L,Q)
%DIRKIN_TWOLINK Dati le lunghezze dei due bracci e i due angoli
%q1 e q2, ottengo la matrice di rototraslazione che caratterizza
%la cinematica diretta del robot planare 2R

%Costruisco la matrice in base alla convenzione di DH
DH = [L(1) 0 0 Q(1) 0 ; L(2) 0 0 Q(2) 0];
T = dirkin(DH);
end

