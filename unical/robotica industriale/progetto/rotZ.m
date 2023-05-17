%%Costruisce una matrice di rotazione attorno all'asse z in tre
%%dimensioni che ruota di un angolo theta in radianti

function R = rotZ(theta)

s = sin(theta);
c = cos(theta);

R = [ c -s 0 ; s c 0 ; 0 0 1 ];

end
