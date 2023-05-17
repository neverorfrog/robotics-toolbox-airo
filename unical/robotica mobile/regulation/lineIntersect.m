function [r] = lineIntersect(xline,yline,xline2,yline2)

%rette entrambe verticali
if xline(2) == xline(1) && xline2(2) == xline2(2)
    r = 0; return;
end

if xline(2) ~= xline(1) && xline2(2) ~= xline2(1)
    
    a = (yline(2) - yline(1))/(xline(2)-xline(1));
    b = yline(1) - xline(1)*a;
    c = (yline2(2)-yline2(1))/(xline2(2)-xline2(1));
    d = yline2(1) - xline2(1)*c;
    x = (d-b)/(a-c);
    r = x*a + b == x*c + d;
    
elseif xline2(2) == xline2(1) && xline(2) ~= xline(1)
    
    x = xline2(1);
    a = (yline(2) - yline(1))/(xline(2)-xline(1));
    b = yline(1) - xline(1)*a;
    y = x*a + b;
    
    %check: il punto di intersezione appartiene alla retta
    r = y > min(yline2) && y < max(yline2);
    %check: il punto di intersezione non é uno dei due vertici della prima retta (di
    %interesse specifico per il metodo di decomposizione in celle)
    r = r && xline(1) ~= x && xline(2) ~= x;
    hi = 2;
else
    
    x = xline(1);
    c = (yline2(2) - yline2(1))/(xline2(2)-xline2(1));
    d = yline2(1) - xline2(1)*c;
    y = x*c + d;
    r = y > min(yline) && y < max(yline);
    r = r && xline2(1) ~= x && xline2(2) ~= x;
    
end

end
