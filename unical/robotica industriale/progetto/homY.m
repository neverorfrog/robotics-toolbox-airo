%%Costruisce una matrice di rototraslazione a partire da una
%%rotazione elementare attorno all'asse y e uno spostamento s
%%dell'origine della terna di arrivo rispetto all'origine della terna di partenza

function RT = homY(theta,s)

RT = [ rotY(theta) s ; 0 0 0 1 ];

end
