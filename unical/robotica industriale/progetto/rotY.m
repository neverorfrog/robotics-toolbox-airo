%%Costruisce una matrice di rotazione attorno all'asse y in tre
%%dimensioni che ruota di un angolo theta in radianti

function R = rotY(theta)

s = sin(theta);
c = cos(theta);

R = [c 0 -s ; 0 1 0 ; s 0 c];

end
