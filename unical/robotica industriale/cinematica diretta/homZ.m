%%Costruisce una matrice di rototraslazione a partire da una
%%rotazione elementare attorno all'asse z e uno spostamento s
%%dell'origine della terna di arrivo rispetto all'origine della terna di partenza

function RT = homZ(theta,s)

RT = [ rotZ(theta) s ; 0 0 0 1 ];

end
