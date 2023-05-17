function dxdt = doppiointegratore(t,x)

u = 1;
x1 = x(1);
x2 = x(2);
dxdt = [0 ; 0];

dxdt(1) = x2;
dxdt(2) = u;
end
