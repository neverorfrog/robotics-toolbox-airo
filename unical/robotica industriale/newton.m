function root = newton(f,x)

eps = 0.001;

% guess iniziale
x0 = x(1) + rand(1)*(x(2) - x(1));
e = 20;

while abs(feval(f,x0)) > eps || e > eps
    
%     derivata nel punto x0
    fdot = diff(sym(f)); fdot = subs(fdot,x0);
%     valuto la funzione nel punto x0
    fx0 = feval(f,x0);
%     risolvo l'equazione cosí da trovare il punto successivo
    x1 = x0 - fx0/fdot; fx1 = feval(f,x1);
%     aggiorno l'errore e x0
    e = fx1 - fx0; x0 = x1;
end

root = eval(x0);
