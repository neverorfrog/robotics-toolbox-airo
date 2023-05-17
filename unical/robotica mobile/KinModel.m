function Pdot = KinModel(t,X,R,wAlpha)

%Dati specifici al modello Differential Drive
r = 0.05; L = 0.15;
K = [r/2 r/2 ; r/L -r/L]; %Per passare da wrwl a v e w

%Estraggo le componenti del vettore stato che mi vengono fornite nella routne ode45
%come condizioni iniziali al problema di Cauchy
x = X(1); y = X(2); theta = X(3);

w = @(t) (wAlpha);
alpha = @(t) (w(t) * t);
xdot_circ = @(t) (-R*sin(alpha(t))*wAlpha);
ydot_circ =  @(t) (R*cos(alpha(t))*wAlpha);
v = @(t) (sqrt(xdot_circ(t)^2 + ydot_circ(t)^2));
wRwL =@(t) (K \ [v(t) ; w(t)]);

%Equazione differenziale effettiva da risolvere poi con ode45
Pdot = ([cos(theta) 0 ; sin(theta) 0 ; 0 1] * K * wRwL(t));

end
