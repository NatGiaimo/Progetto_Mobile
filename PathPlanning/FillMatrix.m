function m = FillMatrix(m,goal)
lim=[size(m,1),size(m,2)];
% viene posto nella queue la cella sulla quale vanno controllati i vicini e
% il valore che deve essere assegnato agli stessi
m(goal(1),goal(2))=1;
queue=[];
n=Neighbours(goal,2,lim);
queue=Push(queue,n);
while ~isempty(queue)
    [queue,point]=Pop(queue);
    if (m(point(1),point(2))==Inf) || (m(point(1),point(2))~= 0) %skip punto se è un ostacolo o è già stato visitato
        continue;
    end
    m(point(1),point(2))=point(3);  % assegnazione valore
    n=Neighbours(point(1:2),point(3)+1,lim);
    queue=Push(queue,n);
end

end

