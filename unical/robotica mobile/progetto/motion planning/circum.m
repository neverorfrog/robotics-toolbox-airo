function [k,l] = circum(temp)

%data una griglia binaria, temp (quaadrato 3x3) é l'intorno di un generico punto di questa griglia

%funzione che calcola quando mi devo spostare all'interno di un quadrato 3x3 avendo
%come obiettivo quello di circumnavigare l'ostacolo. Oriento la direzione del robot
%quindi in base a che posizione nell'intorno si trovano i valori inf.

%sfrutto la variabile global verso che mi indica se girare in senso orario o antiorario
global verso;

%l é l'incremento di ascissa e k l'incremento di ordinata, ma a entrambi va sottratto
%2. Quindi l = 3 é un aumento di uno, l = 1 torna indiestro di uno.

destra = [1,0]; sinistra = [-1,0]; sopra = [0,1]; sotto = [0,-1];
ris = [];

%mi muovo a sinistra(orario) o destra(antiorario) - lato di sotto
if isempty(find(~temp == [0 0 0;0 0 0;1 1 1])) ||...
        isempty(find(~temp == [0 0 0;0 0 0;1 1 0])) ||...
        isempty(find(~temp == [0 0 0;0 0 0;0 1 1])) 
    if verso == "orario"
        ris = sinistra;
    elseif verso == "antiorario"
        ris = destra;
    end
end
%angolo sotto a sinistra di un ostacolo in mezzo allo spazio
if isempty(find(~temp == [0 0 0;0 0 0;0 0 1]))
     if verso == "orario"
        ris = sopra;
    elseif verso == "antiorario"
        ris = destra;
    end
end

%mi muovo a destra(orario) o sinistra(antiorario) - lato di sopra
if isempty(find(~temp == [1 1 1;0 0 0;0 0 0])) ||...
        isempty(find(~temp == [1 1 0;0 0 0;0 0 0])) ||...
        isempty(find(~temp == [0 1 1;0 0 0;0 0 0]))
    if verso == "orario"
        ris = destra;
    elseif verso == "antiorario"
        ris = sinistra;
    end
end
%angolo sopra a destra di un ostacolo in mezzo allo spazio
if isempty(find(~temp == [1 0 0;0 0 0;0 0 0]))
    if verso == "orario"
        ris = sotto;
    elseif verso == "antiorario"
        ris = sinistra;
    end
end

%mi muovo verso sopra(orario) o sotto(antiorario) - lato sinistro
if isempty(find(~temp == [0 0 1;0 0 1;0 0 0])) ||...
        isempty(find(~temp == [0 0 0;0 0 1;0 0 1])) ||...
        isempty(find(~temp == [0 0 1;0 0 1;0 0 1]))
    if verso == "orario"
        ris = sopra;
    elseif verso == "antiorario"
        ris = sotto;
    end
end
%angolo sopra a sinistra di un ostacolo in mezzo allo spazio
if isempty(find(~temp == [0 0 1;0 0 0;0 0 0]))
     if verso == "orario"
        ris = destra;
    elseif verso == "antiorario"
        ris = sotto;
    end
end

%mi muovo verso sotto(orario) o sopra(antiorario) - lato destro
if isempty(find(~temp == [0 0 0;1 0 0;1 0 0])) ||...
        isempty(find(~temp == [1 0 0;1 0 0;0 0 0])) ||...
        isempty(find(~temp == [1 0 0;1 0 0;1 0 0]))
    if verso == "orario"
        ris = sotto;
    elseif verso == "antiorario"
        ris = sopra;
    end
end
%angolo sotto a destra di un ostacolo in mezzo allo spazio
if isempty(find(~temp == [0 0 0;0 0 0;1 0 0]))
    if verso == "orario"
        ris = sinistra;
    elseif verso == "antiorario"
        ris = sopra;
    end
end

%vicolo cieco sopra destra
if isempty(find(~temp == [0 0 1 ; 0 0 1 ; 1 1 1]))
     if verso == "antiorario" %arrivo da sinistra e devo scendere
        ris = sotto;
    elseif verso == "orario"%arrivo da sotto e devo andare a sinistra
        ris = sinistra;
    end
end

%vicolo cieco sotto destra
if isempty(find(~temp == [1 1 1 ; 0 0 1 ; 0 0 1]))
    if verso == "antiorario" %arrivo da sopra e devo andare a sinistra
        ris = sinistra;
    elseif verso == "orario"%arrivo da sinistra e devo salire
        ris = sopra;
    end
end

%vicolo cieco sopra sinistra
if isempty(find(~temp == [1 0 0 ; 1 0 0 ; 1 1 1]))
     if verso == "antiorario" %arrivo da sotto e devo andare a destra
        ris = destra;
    elseif verso == "orario"%arrivo da destra e devo scendere
        ris = sotto;
    end
end

%vicolo cieco sotto sinistra
if isempty(find(~temp == [1 1 1; 1 0 0;1 0 0]))
     if verso == "orario" %arrivo da sopra e devo andare a destra
        ris = destra;
    elseif verso == "antiorario"%arrivo da destra e devo salire
        ris = sopra;
    end
end

l = ris(1)+2; k = ris(2)+2;
    

