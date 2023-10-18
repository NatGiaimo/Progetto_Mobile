function via = VoronoiDiag(E,O,start,goal)
%% costruzione diagramma di voronoi
%ricaviamo una rappresentazione degli ostacoli avendo una discretizzazione
%dei loro lati. Aggiungiamo anche una descrizione dell'ambiente
rect=cell(size(O,1)+1,1);
for i=1:size(O,1)
    rect{i}=obstacleToShape(O{i},0.1);
end
rect{end}=obstacleToShape(E,0.1);
x=[];
y=[];
%poniamo tutte le coordinate in due vettori colonna x e y
for i=1:size(rect,1)
   for j=1:size(rect(i),1)
       x=[x;rect{i}(:,1)];
       y=[y;rect{i}(:,2)];
   end
end

[v,~]=voronoin([x,y]);

%bisogna rimuovere i riferimenti interni agli ostacoli e i punti esterni al
%perimetro
for i=1:size(O,1)
    in=inpolygon(v(:,1),v(:,2),O{i}(1,:),O{i}(2,:));
    v(in,:)=NaN;
end
out=~inpolygon(v(:,1),v(:,2),E(1,:),E(2,:));
v(out,:)=NaN;
%puliamo v da ogni riga che abbia almeno un NaN
v=v(~any(isnan(v),2), :);
clear out;

%% path

%l'id del nodo corrisponderà all'indice del vertice in v
g=graph(adjMatrix(v),'omitselfloops');

%identifichiamo i vertici più vicino al punto di goal e di start
distList=dist(v(:,1),v(:,2),goal(1),goal(2));
[~, indG]=min(distList(:));
distList=dist(v(:,1),v(:,2),start(1),start(2));
[~, indS]=min(distList(:));

%calcolo del percorso a minima distanza tra i vertici identificati
indPath=shortestpath(g,indS,indG);

via=v(indPath,:);

%forziamo l'ultimo e il primo punto
via=[start;via;goal];

figure('name','Voronoi');
plot(rect{end}(:,1),rect{end}(:,2)); hold on;
plot([start(1),goal(1)],[start(2),goal(2)],'x'); hold on;
for i=1:size(O,1)
    plot(rect{i}(:,1),rect{i}(:,2)); hold on;
end
plot(v(:,1),v(:,2),'+b'); hold on;
plot(via(:,1),via(:,2),'-r'); hold on;
hold off;

end
%Calcola la distanza tra un set di punti [X Y] e un punto [x y]
function f=dist(X,Y,x,y)
    f=sqrt((x-X).^2+(y-Y).^2);
end

function m=adjMatrix(v)

m=zeros(size(v,1),size(v,1));

for i=1:size(v,1)
    curr=v(i,:);
    %salviamo la distanza tra il vertice corrente e tutti i vertici in v
    distList=dist(v(:,1),v(:,2),curr(1),curr(2));
    %riempiamo la matrice con tutti i valori minori di una costante, in
    %questo caso 0.5 [m]
    %La costante rappresenta la massima distanza per la quale venogno
    %create le connessioni nel grafo. 
    for j=1:size(distList,1)
        if distList(j)>(0.5)
           continue; 
        end
        %assicuriamo che m sia simmetrica
        m(i,j)=distList(j);
        m(j,i)=distList(j);
    end
end
end

