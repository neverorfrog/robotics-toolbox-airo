function [J,singularities] = jacobian_ant(Q,R,A)

% Procedimento analitico
syms("q",[3,1]);
P = R(1:3,4); a2 = A(2); a3 = A(3);
persistent sing; persistent Js; persistent eq;

if isempty(Js)
    Js = sym([]);
    
    for i = 1:3
        for j = 1:3
            Js(i,j) = diff(P(i),q(j));
        end
    end
    
    eq = simplify(det(Js)) == 0;
    sing = solve(eq,[q1 q2 q3]);
end

% Procedimento geometrico

% if isempty(Js)
%     Js = sym([]);
%     
% % Assi z dei giunti
%     z0 = antisimm([0 0 1]);
%     z1 = antisimm([sin(q1) -cos(q1) 0]);
%     z2 = z1;
%   
% %     Posizioni delle origini delle terne rispetto a SR0
%     p0 = [0 0 0]'; p1 = [0 0 0]';
%     p2 = [a2*cos(q1)*cos(q2) ; a2*sin(q1)*cos(q2) ; a2*sin(q2)];
%     p3 = R(1:3,4);
%     
%     Js(:,1) = z0*(p3-p0); 
%     Js(:,2) = z1*(p3-p1);
%     Js(:,3) = z2*(p3-p2);
%     
% end

singularities = sing;
J = subs(Js,[q1 q2 q3],[Q(1) Q(2) Q(3)]);


end
