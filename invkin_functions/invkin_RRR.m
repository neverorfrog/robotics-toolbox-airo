%Cinematica inversa di un manipolatore antropomorfo (elbow-type arm)

function Q = invkin_ant(P,L,d1)

    %% Dati in input
    px = P(1); py = P(2); pz = P(3);
    L2 = L(2); L3 = L(3);
    
    %% Dichiarazione variabili simboliche
    syms("c",[3,1]); syms("s",[3,1]);
    
    %% theta3
    cos3 = (px^2 + py^2 + (pz-d1)^2 - L2^2 - L3^2)/(2*L2*L3);
    sin3 = [sqrt(1-cos3^2) ; ... %sin > 0: elbow down 
            -sqrt(1-cos3^2)];    %sin < 0: elbow up  
    theta3 = atan2(sin3,cos3);
    
    %% theta1
    theta1 = [atan2(py,px) ; ...%right
              atan2(-py,-px)]; %left
    sin1 = sin(theta1);
    cos1 = cos(theta1);
    
    %% theta2
    
    A = []; b = [];
    A{1} = [(L2+L3*cos3) , (-L3*sin3(1)) ; L3*sin3(1) , L2+L3*cos3];
    A{2} = A{1};
    A{3} = [(L2+L3*cos3) , (-L3*sin3(2)) ; L3*sin3(2) , L2+L3*cos3];
    A{4} = A(:,:,3);
    b{1} = [cos1(1)*px + sin1(1)*py ; pz];
    b{3} = b(:,:,1);
    b{2} = [cos1(2)*px + sin1(2)*py ; pz];
    b{4} = b(:,:,2);
    
    for i = 1 : 4
        temp = A{i} \ b{i};
        cos2(i,1) = temp(1); 
        sin2(i,1) = temp(2);
    end
    theta2 = atan2(sin2,cos2);
    
    %% Output possibili
    %Avró quattro terne di variabili di giunto, di cui ognuna corrisponde a una diversa
    %configurazione di gomito del braccio
    
    Q{1} = [theta1(1)  theta2(1)  theta3(1)];
    Q{2} = [theta1(2)  theta2(2)  theta3(1)];
    Q{3} = [theta1(1)  theta2(3)  theta3(2)];
    Q{4} = [theta1(2)  theta2(4)  theta3(2)];

end
