function f = obstacleToShape(obstacle,disc)
%obstacle(i,j) con i in [x,y] e j identificatore vertice
    % lato dal IV al I quadrante, movimento positivo sulle y
    d=obstacle(2,4):disc:obstacle(2,1);
    IVtoI=zeros(length(d),2);
    for i=1:length(d)
       IVtoI(i,:)=[ obstacle(1,4) d(i) ];
    end
    clear d;
    
    % lato dal I al II quadrante, movimento negativo sulle x
    d=obstacle(1,1):-disc:obstacle(1,2);
    ItoII=zeros(length(d),2);
    for i=1:length(d)
       ItoII(i,:)=[d(i) obstacle(2,2)];
    end
    clear d;
    
    %lato dal II al III quadrante, movimento negativo sulle y
    d=obstacle(2,2):-disc:obstacle(2,3);
    IItoIII=zeros(length(d),2);
    for i=1:length(d)
       IItoIII(i,:)=[ obstacle(1,2) d(i) ];
    end
    clear d;
    
    %lato dal III al IV quadrante, movimento positivo sulle x
    d=obstacle(1,3):disc:obstacle(1,4);
    IIItoIV=zeros(length(d),2);
    for i=1:length(d)
       IIItoIV(i,:)=[ d(i) obstacle(2,3) ]; 
    end

    f=cat(1,IVtoI,ItoII,IItoIII,IIItoIV);
end

