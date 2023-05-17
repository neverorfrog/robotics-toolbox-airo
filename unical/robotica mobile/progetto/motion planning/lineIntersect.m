% Se i due segmenti si intersecano, la funzione restituisce true

function r = lineIntersect(xline,yline,xline2,yline2)

xlinei = xline(1); xlinef = xline(2); ylinei = yline(1); ylinef = yline(2);
xline2i = xline2(1); xline2f = xline2(2); yline2i = yline2(1); yline2f = yline2(2);

syms x y;

eps = 0.001;

if abs(xlinef - xlinei) > eps && abs(ylinef - ylinei) > eps
    line1 = (x - xlinei)/(xlinef - xlinei) == (y-ylinei)/(ylinef-ylinei);
elseif abs(xlinef - xlinei) < eps && abs(ylinef - ylinei) > eps
    line1 = x == (xlinei+xlinef)/2;
elseif abs(xlinef - xlinei) > eps && abs(ylinef - ylinei) < eps
    line1 = y == (ylinei+ylinef)/2;
end

if abs(xline2f - xline2i) > eps && abs(yline2f - yline2i) > eps
    line2 = (x - xline2i)/(xline2f - xline2i) == (y-yline2i)/(yline2f-yline2i);
elseif abs(xline2f - xline2i) < eps && abs(yline2f - yline2i) > eps
    line2 = x == (xline2i+xline2f)/2;
elseif abs(xline2f - xline2i) > eps && abs(yline2f - yline2i) < eps
    line2 = y == (yline2i+yline2f)/2;
end


system = [line1 line2];
sol = vpasolve(system);
if isempty(sol)
    r = 0;
else
    x = sol.x; y = sol.y;
    c1 = x > min(xline) && x < max(xline) && y > min(yline) && y < max(yline);
    c2 = y > min(yline2) && y < max(yline2);
    if c1 && c2
        r = 1;
    else
        r = 0;
    end
end

end
