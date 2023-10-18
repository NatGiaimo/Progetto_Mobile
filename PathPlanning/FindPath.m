function path = FindPath(matrix,start,goal)
lim=[size(matrix,1),size(matrix,2)];
%lim non è necessario ma ricicliamo la funzione Neighbours
%da qualsiasi punto si parta, bisogna seguire il percorso a costo minimo
path=[start];
n=Neighbours(start,0,lim);
queue=n;
dist=@(p1,p2)((p1(1)-p2(1)).^2+(p1(2)-p2(2)).^2);
while ~isempty(queue)
    [queue,point]=Pop(queue);
    prevPoint=path(end,:);    %rapido accesso all'ultimo punto visitato
    d1=dist(point,goal);
    if ( matrix(prevPoint(1),prevPoint(2)) > matrix(point(1),point(2)) )
        %controlliamo se i vicini restanti hanno distanza minore dal goal,
        %questo serve per evitare che la path prenda il primo punto trovato
        %che abbia valore minore di prevPoint. 
        queue2=queue;
        while ~isempty(queue2)
            [queue2,p]=Pop(queue2);
            d2=dist(p,goal);
            if matrix(p(1),p(2)) > matrix(point(1),point(2))
                continue;
            end
            %se uno dei vicini restanti ha valore minore uguale al corrente
            %e distanza minore al goal, allora esso diventa il novo punto
            %della path.
            if d2<d1 && ( matrix(p(1),p(2)) <= matrix(point(1),point(2)) )
               point=p;
               break;
            end
        end
        path=[path;point(1:2)];
        n=Neighbours(point,0,lim);
        queue=n;
    end
end

end

