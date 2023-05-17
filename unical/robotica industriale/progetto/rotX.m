%%Costruisce una matrice di rotazione attorno all'asse x in tre
%%dimensioni che ruota di un angolo theta in radianti

function R = rotX(theta)

s = sin(theta);
c = cos(theta);

R = [ 1 0 0 ; 0 c -s ; 0 s c];

end


