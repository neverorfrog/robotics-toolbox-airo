function [P]=cin_dir(L,Q)

L1=0;
L2=0.75;
L3=0.75;
L = [L1; L2; L3]; %vettore colonna
Q = [q1; q2; q3;] %vettore variabili di giunto

% Tabella dei parametri di Denavit-Hartenberg robot
a1=0;  d1=0; tw1=pi/2; th1=q1; 
a2=L2; d2=0; tw2=0;    th2=q2;
a3=L3; d3=0; tw3=0;    th3=q3;

%notazioni utili
c1=cos(Q(1));
c2=cos(Q(2));
c23=cos(Q(2)+Q(3));
s1=sin(Q(1));
s2=sin(Q(2));
s23=sin(Q(2)+Q(3));

%Da Tabella di Denavit-Hartenberg costruisco le Matrici di RotoTralaszione
%che legano i vari sistemi di riferimenti posti sul robot in esame e li
%racchiudo all'interno del 'matricione' Tdir:

Tdir= [ c1*c23 -c1*s23 s1 c1*(a2*c2+a3*c23);
    s1*c23 -s1*s23 -c1 s1*(a2*c2+a3*c23);
    s23 c23 0 (a2*s2+a3*s23);
    0 0 0 1]

%data la matrice di rototraslazione che lega il Sistema di Riferimento 0
%con quello Base, cosi data:

R = [ 1 0 0 0.5;
    0 1 0 0.5;
    0 0 1 1;
    0 0 0 1]

%posso determinare la Cinematica Diretta:

Tdir_eff = R*Tdir;

P =[Tdir_eff(1,4); Tdir_eff(2,4); Tdir_eff(3,4)]


end







