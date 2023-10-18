function via = DPF(E,O,start,goal)
%% Discretizzazione enviroment
delta=0.1;
xx=E(1,3):delta:E(1,1);
yy=E(2,3):delta:E(2,1);
[XX,YY]=meshgrid(xx,yy);
g=pointToGrid(goal,delta);
s=pointToGrid(start,delta);
envMatrix=zeros(size(XX,1),size(XX,2));

%% Inf sulle celle che contengono ostacoli
inp=zeros(size(XX,1),size(XX,2));
for i=1:size(O,1)
    inp=inp+inpolygon(XX,YY,O{i}(1,:),O{i}(2,:));
end

for i=1:size(XX,1)
   for j=1:size(XX,2)
      if inp(i,j)==1
         envMatrix(i,j)=Inf; 
      end
   end
end
clear inp;
%% Riempimento celle

envMatrix=FillMatrix(envMatrix,g);

%% risoluzione percorso

viaGrid=FindPath(envMatrix,s,g);
via=zeros(size(viaGrid,2),2);
for k=1:size(viaGrid)
    via(k,:)=gridToPoint(viaGrid(k,:),delta);
    envMatrix(viaGrid(k,1),viaGrid(k,2))=NaN;
end

figure('name','path');
plotMap(envMatrix);
end
%% funzioni
function f=plotMap(envMatrix)
%crediti: https://it.mathworks.com/matlabcentral/answers/216568-recolor-imagesc-inf-values-in-matlab
infRGB = reshape([0 0 0],1,1,3);
%set to RGB triple for desired color for inf values in this case black
%denoted by [0 0 0].
infimgdata = repmat(infRGB, size(envMatrix,1), size(envMatrix,2));
%same size as "a" but all the one color
image(infimgdata, 'alphadata', ~isnan(envMatrix));
%plots an image based on 'infimgdata' excludes NaN's therefore making NaN
%values white
hold on
%plot to same figure
imagesc(envMatrix,'alphadata', ~(isnan(envMatrix)|isinf(envMatrix)));
set(gca,'YDir','normal')
%plot data
colormap(jet());
colorbar
grid on
hold off
% stop plotting to the same figure
end

function p=gridToPoint(point,delta)
    x=(point(2)-1)*(delta);
    y=(point(1)-1)*(delta);
    p=[x y];
end

function p=pointToGrid(point,delta)
%viene aggiunto 1 poiché gli indici su matlab partono da 1
    i=point(1)/delta+1;
    j=point(2)/delta+1;
    p=[j i];
    % l'asse delle x divide lo spazio in colonne
end