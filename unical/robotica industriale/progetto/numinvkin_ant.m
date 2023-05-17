function Q = numinvkin_ant(P,A,Q1)

%% Dati in input
P = P(:);
a2 = A(2); a3 = A(3);

%% Jacobiano simbolico
syms q1 q2 q3;
Js = [-sin(q1)*(a2*cos(q2) + a3*cos(q2+q3)) , -cos(q1)*(a2*sin(q2) + a3*sin(q2+q3)) , -a3*cos(q1)*sin(q2+q3) ;...
    cos(q1)*(a2*cos(q2) + a3*cos(q2+q3)) , -sin(q1)*(a2*sin(q2) + a3*sin(q2+q3)) , -a3*sin(q1)*sin(q2+q3) ;...
    0 , a2*cos(q2) + a3*cos(q2+q3) , a3*cos(q2+q3)];

%% Procedimento per metodo di newton

e = 20;
k = 0.01;

while norm(e)^2 > 0.001
    Q0 = Q1;
    % Innanzitutto valuto lo jacobiano in q0
    Jq0 = subs(Js,[q1 q2 q3],[Q0(1) Q0(2) Q0(3)]);
    
    %Posa calcolata in q0
    Rq0 = dirkin_ant(Q0,A); 
    fq0 = Rq0(1:3,4);
    
    %errore al passo k
    e = P - fq0;
    
     %calcolo il nuovo q
    Q1 = simplify(Q0 + 0.01*Jq0 * e); 
end

Q = Q1;




