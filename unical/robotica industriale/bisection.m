function root = bisection(f,x,varargin)

% scelgo i primi due punti all'interno di tx
xk1 =  pi/2*rand(1);
xk2 = pi/2*rand(1);
gk1 = abs(feval(f,xk1)); gk2 = abs(feval(f,xk2));
e = min([gk1,gk2]);

% faccio partire la ricerca dello zero a partire da questi due punti
while e > 0.00001
    temp = xk2;
    xk2 = ( gk2/(gk2-gk1) + xk2/(xk1-xk2) ) * (xk1 - xk2);
    xk1 = temp; gk1 = gk2;
    gk2 = abs(feval(f,xk2));
    
    e = gk2;
    
end

root = xk2;

end
