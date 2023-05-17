function T = dirkin_ant(Q,L)

persistent Ts; persistent DH;

syms("q",[3 1]);

if isempty(Ts) 
    DH = [0 pi/2 0 q1; L(2) 0 0 q2; L(3) 0 0 q3];
    Ts = simplify(dirkin(DH));
end

if ~(symType(sym(Q)) == "variable")
    T = double(subs(Ts,[q1 q2 q3],[Q(1) Q(2) Q(3)]));
else
    T = Ts;
end

end
