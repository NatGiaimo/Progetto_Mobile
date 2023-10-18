function via = VisibilityGraph(E,O,s,g)
%% creazione segmenti tra i vertici
%gli ostacoli sono ingrassati prima che le coordinate vengano passate a
%questa funzione, il robot può navigare sul perimetro descritto dai vertici
%in O senza collisioni.
x=[]; y=[];
%poniamo le coordiante dei vertici di ostacoli e ambiente in due vettori
%colonna in modo che ci sia corrispondenza degli elementi sullo stesso
%indice
for i=1:size(O,1)
    x=[x; O{i}(1,:)'];
    y=[y; O{i}(2,:)'];
end
%aggiungiamo le coordinate dell'enviroment, start e goal 
x=[x;E(1,:)';s(1);g(1)];
y=[y;E(2,:)';s(2);g(2)];

%rimuoviamo da x e y i punti esterni all'enviroment e interni agli ostacoli
[in,~]=inpolygon(x,y,E(1,:),E(2,:));
x(~in)=NaN; y(~in)=NaN;
%per gli ostacoli
for i=1:size(O,1)
    [in,on]=inpolygon(x,y,O{i}(1,:),O{i}(2,:));
    %rimuoviamo solo i punti interni, non i vertici
    x(in & ~on)=NaN; y(in & ~on)=NaN;
end
clear in;
%puliamo x e y da ogni NaN
x=x(~any(isnan(x),2));
y=y(~any(isnan(y),2));

lines={size(x,1)};
%lines{i} conterrà una matrixe Nx2, sulle colonne le coordinate [x,y] dei
%vertici che creano segmenti con [x(i),y(i)] che non intersecano ostacoli.

%creiamo i segmenti iterando su ogni vertice e controllando le intersezioni
%con gli ostacoli
for i=1:size(x,1)
    lines{i}=[];
    for j=1:size(x,1)
        currLine=[x(i) x(j); y(i) y(j)];
        %controlliamo se esistono intersezioni tra gli ostacoli e il
        %segmento corrente
        coords=double.empty(0,2);
        %usiamo flag per indicare se il segmento è la diagonale di un
        %poligono
        flag=0;
        for k=1:size(O,1)
            flag=isDiagonal(O{k},currLine);
            if flag==1
                break;
            end
            [xi,yi]=polyxpoly([O{k}(1,:),O{k}(1,1)],[O{k}(2,:),O{k}(2,1)],currLine(1,:),currLine(2,:));
            %il primo vertice è ripetuto, altrimenti la funzione non tiene
            %conto del segmento tra i vertici del IV e del I quadrante
            %(riferimento spiegato in obstacles.m)
            coords=[coords;xi,yi];
        end
        if flag==1
            continue;
        end
        %rimuoviamo i punti di intersezione che ricadono esattamente sui
        %vertici
        
        coords=setdiff(coords,[x,y],'rows');
        
        if ~any(coords)  %memorizza le coordinate se non esistono intersezioni
            lines{i}=[lines{i};currLine(1,2),currLine(2,2)];
        end
    end
end

%% path
    %creiamo un grafo con nodi i vertici e peso pari alla loro distanza
    gr=graph(adjMatrix(x,y,lines),'omitselfloops');
    
    %start e goal sono rispettivamente penultimo e ultimo elemento di [x,y]
    indPath=shortestpath(gr,size(x,1)-1,size(x,1));
    
    via=[x(indPath),y(indPath)];
    
    
    plot([E(1,:),E(1,1)],[E(2,:),E(2,1)],'-');
    hold on
    for j=1:size(O,1)
        plot([O{j}(1,:),O{j}(1,1)],[O{j}(2,:),O{j}(2,1)]);
        hold on;
    end
    plot(s(1),s(2),'-p');
    hold on;
    plot(g(1),g(2),'-p');
    hold on;
    for i=1:size(lines,2)
       for j=1:size(lines{i},1)
           plot([x(i),lines{i}(j,1)],[y(i),lines{i}(j,2)],'-r');
           hold on;
       end
    end
    plot(via(:,1),via(:,2),'-g');
    axis equal;



end
%% funzioni

function f=isDiagonal(O,points)
    x=linspace(points(1,1),points(1,2),3);
    y=linspace(points(2,1),points(2,2),3);
    %controlliamo se il punto medio si trova all'interno di un ostacolo
    [in,on]=inpolygon(x(2),y(2),O(1,:),O(2,:));
    if any(in) && ~any(on)
        f=1;
    else
        f=0;
    end
end

function m=adjMatrix(x,y,lines)
dist=@(X,Y,x,y)(sqrt((x-X).^2+(y-Y).^2));
    m=zeros(size(x,1),size(x,1));
    for i=1:size(x,1)
        for j=1:size(lines{i},1)
            xl=lines{i}(j,1); yl=lines{i}(j,2);
            d=dist(x(i),y(i),xl,yl);
            %ricaviamo l'indice del vertice contenuto in lines{i}(j,:)
            index=find(x==xl & y==yl);
            m(i,index)=d;
            m(index,i)=d;
        end
    end

end
