%%Costruisce una matrice di rototraslazione a partire da una
%%rotazione elementare attorno all'asse x e uno spostamento s
%%dell'origine della terna di arrivo rispetto all'origine della terna di partenza

function RT = homX(theta,s)

RT = [ rotX(theta) s ; 0 0 0 1 ];

end
