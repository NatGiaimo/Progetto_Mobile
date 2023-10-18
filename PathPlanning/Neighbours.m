function n = Neighbours(cell,val,lim)
%PRE: cell=[x,y], x,y>0
%   lim=[x,y], x,y massimo valore della matrice enviroment

sp=cell(:)-1;   %starting point
n=[];  %coda Nx2, dove N sono i vicini "legali" di cell
for i=0:2
   for j=0:2
       x=sp(1)+i; y=sp(2)+j;
       if x<=0 || y<=0    % salto iterazione se uno dei punti è fuori dalla matrice o è la cella "generatrice"
           continue;
       elseif x>lim(1) || y>lim(2)
           continue;
       elseif (x==cell(1))&&(y==cell(2))
           continue;
       end
       n=Push(n,[x y val]); %aggiunta in coda del punto corrente
   end
end
end
