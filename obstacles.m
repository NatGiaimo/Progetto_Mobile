function [O,E] = obstacles(dBot)
    %E = enviroment, matrice 4x2 che rappresenta i 4 angoli della "stanza"
    %partendo da quello in alto a destra (seguendo la convenzione dei
    %quadranti) e proseguendo in senso anti orario.
    %Sulle colonne indici dei punti, sulle righe coordinata x e y
    %O = vettore di dimensione N che contiene le matrici che rappresentano
    %gli ostacoli
        %obstacle=ostacolo, matrice 4x2 che segue la stessa
        %rappresentazione dell'enviroment.
        
    %Prevediamo una stanza 5x5[m] con un robot DD dBotxdBot[cm]
    %tre ostacoli di forma rettangolare (per comodità)
    
    E=[[5 0 0 5];
        [5 5 0 0]];

    o1=[[1 0 0 1];
        [5 5 2.5 2.5]];
    
    o2=[[3 2 2 3];
        [3 3 0 0]];
    
    o3=[[5 2 2 5];
        [5 5 4 4]];
    
    o4=[[5 4 4 5]
        [2 2 1.5 1.5]];
    
    O=cell(3,1);
    O{1}=o1; O{2}=o2; O{3}=o3; O{4}=o4;
    %ingrassamento degli ostacoli di fattore dBot.
    %Il centro del robot deve potersi muovere sui bordi dell'ostacolo
    
    %sommiamo e sottraiamo la dimensione del robot in base al vertice che
    %stiamo analizzando, si segue la convenzione sopra specificata
    for i=1:size(O,1)
        %immaginando una divisione in quadranti dell'ostacolo:
        %O(nOstacolo,[x,y],nPunto)
        %I quadrante
        O{i}(1,1)=O{i}(1,1)+dBot;   %x
        O{i}(2,1)=O{i}(2,1)+dBot;   %y
        %II quadrante
        O{i}(1,2)=O{i}(1,2)-dBot;   %x
        O{i}(2,2)=O{i}(2,2)+dBot;   %y
        %III
        O{i}(1,3)=O{i}(1,3)-dBot;   %x
        O{i}(2,3)=O{i}(2,3)-dBot;   %y
        %IV
        O{i}(1,4)=O{i}(1,4)+dBot;   %x
        O{i}(2,4)=O{i}(2,4)-dBot;   %y
    end
    
end
