function ris = prova(d)

    persistent cont; 
    if isempty(cont)
        cont = 1;
    end
    
    
    if mod(d,2) == 0
        cont = cont+1
    end
