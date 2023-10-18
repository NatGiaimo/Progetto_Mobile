function [rt,raggio] = RTsulPiano(P)
    %% Ricaviamo la normale del piano sul quale giacciono i punti P
    % Piano := ax + by + cz + d = 0 con n=[a b c]
    
    % troviamo due vettori che giacciono sul piano
    p1p2=P(:,1)-P(:,2);
    p1p3=P(:,1)-P(:,3);
    % otteniamo la normale al piano come prodotto vettoriale tra i
    % due vettori precedentemente espressi
    n = cross(p1p2, p1p3);
    
    %ricaviamo il centro della circonferenza come il punto medio tra i P
    centro=zeros(3,1);
    for i=1:3
       centro(i)=(P(i,1)+P(i,2)+P(i,3))/3;
    end
    
    raggio=norm(P(:,1)-centro);
    
    %% cambio sistema di riferimento
    
    %sia p il sistema di riferimento del piano e b il sistema di
    %riferimento di base
    
    %a tutti gli assi viene applicata una traslazione per porre il punto
    %'centro' come origine del sistema
    %sia u1 l'asse x su p
    u1=cross(n,[1 0 0]);
    u1=u1+centro';
    
    %sia u2 il vettore perpendicolare alla normale e al vettore u1
    u2=cross(n,u1);
    u2=u2+centro';
    
    %l'asse z corrisponde alla normale del piano
    u3=n;
    u3=u3+centro;
    
    %abbiamo gli assi, troviamo gli angoli
    %esprimiamo i vettori di base del sistema di riferimento b
    b1=[1 0 0];
    b2=[0 1 0];
    b3=[0 0 1];
    %calcoliamo gli angoli compresi tra gli assi del sistema b e il sistema p
    theta = dot(u1, b1)/(norm(u1)*norm(b1));
    gamma = dot(u2, b2)/(norm(u2)*norm(b2));
    omega = dot(u3, b3)/(norm(u3)*norm(b3));
    %costruiamo la matrici di rotazione
    r=eul2r(theta,gamma,omega);
    
    %troviamo il vettore di traslazione da A a B, sarà d=Oa-Ob, con
    %Ob=centro e Oa=[0 0 0], quindi d=-centro
    
    %possiamo, quindi, esprimere la matrice di rototraslazione da b a p
    rt=[r,-centro;0,0,0,1];
    
    plotThis()
    
    function [] = plotThis()
        figure('name','sistema di riferimento sul piano')
        plot3(centro(1),centro(2),centro(3),'O')
        hold on
        
        plot3([centro(1); 0],[centro(2); 0], [centro(3); 0], '-')
        hold on
        
        plot3([centro(1); u1(1)],[centro(2); u1(2)], [centro(3); u1(3)], '-');
        hold on
        plot3([centro(1); u2(1)],[centro(2); u2(2)], [centro(3); u2(3)], '-');
        hold on
        plot3([centro(1); u3(1)],[centro(2); u3(2)], [centro(3); u3(3)], '-');
        hold on

        plot3(P(1,:),P(2,:),P(3,:),'O')
        grid on
    end
end