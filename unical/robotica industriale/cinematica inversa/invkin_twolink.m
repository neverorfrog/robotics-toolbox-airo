%Funzione che estrapola, risolvendo un problema di cinematica inversa, le variabili di
%giunto di un robot planare 2R, date le lunghezze dei due bracci e la posizione
%dell'EE nel piano cartesiano. Q quindi é un vettore di due variabili q1 e q2.

function Q = invkin_twolink(L,P,varargin)

Px = P(1); Py = P(2); %Coordinate del punto da raggiungere
L1 = L(1); L2 = L(2); %Lunghezze dei bracci

%Controllo solvibilitá problema


%Trovo q2
c2 = (Px^2+Py^2 - (L1^2+L2^2))/(2*L1*L2);
%Controllo se c'é il terzo parametro, e se c'é differenzio in base
%al parametro tra soluzione a gomito alto e basso
narginchk(2,3);
if nargin == 3
    if varargin{1} == 0
        s2 = sqrt(1 - c2^2); %Gomito basso
    else
        s2 = -sqrt(1 - c2^2); %Gomito alto
    end
else
    s2 = sqrt(1 - c2^2);
end

if c2 == -1
    s2 = 1; %Se i bracci sono allineati devo stenderlo in alto perpendicolare
end
Q(2) = atan2(s2,c2);

%Trovo q1
A = [L1 + L2*c2 , -L2 * s2 ; 
        L2 * s2      , L1 + L2*c2];
c1s1 = A\P(:);
c1 = c1s1(1); s1 = c1s1(2);
Q(1) = atan2(s1,c1);

end
