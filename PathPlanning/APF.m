function via = APF(E,Obs,s,g,dBot)
%% inizializzazione
    %trasformiamo gli ostacoli da una rappresentazione in soli vertici a
    %una discretizzazione sui lati di passo 0.1[m]
    O={};
    for i=1:size(Obs,1)
       O{i}=obstacleToShape(Obs{i},0.1);
    end
    
    xm=E(1,3);xM=E(1,1);
    ym=E(2,3);yM=E(2,1);
    
    rm=@(theta)([cos(theta) -sin(theta); sin(theta) cos(theta)]);
    
    NJax=@(x,y,Gx,Gy)(x-Gx);
    NJay=@(x,y,Gx,Gy)(y-Gy);
    NJrx=@(x,y,Ox,Oy)(2*(Ox-x)./(((x-Ox).^2+(y-Oy).^2).^2));
    NJry=@(x,y,Ox,Oy)(2*(Oy-y)./(((x-Ox).^2+(y-Oy).^2).^2));
    
    rho=@(x,y,Ox,Oy)((x-Ox).^2+(y-Oy).^2<=dBot^2); 

    wa=2; wo=5;
    delta=0.1; 
    alpha=0.01;
    
%%
%plot campo potenziale
    %a seguito dell'ingrassamento, i vertici degli ostacoli potrebbero
    %risiedere all'esterno dell'ambiente; aggiungiamo 2[m] agli estremi
    %dell'ambiente per prevenire errori.
    xx=xm-2:delta:xM+2;
    yy=ym-2:delta:yM+2;
    [XX,YY]=meshgrid(xx,yy);
    
    NJaXX=NJax(XX,YY,g(1),g(2));
    NJaYY=NJay(XX,YY,g(1),g(2));

    %troviamo l'angolo theta che evita minimi locali con l'enviroment
    theta=zeros(size(O,2));
    for i=1:size(O,2)
       %si calcola l'angolo formato tra il centro dell'ostacolo e i punti
       %di start e goal; scegliendo tra - o + pi/3 se la traiettoria deve
       %essere oraria o anti oraria
       [cx,cy]=centroid(polyshape(Obs{i}(1,:),Obs{i}(2,:)));
       %ricaviamo la pendenza delle rette
       ms=(s(2)-cy)/(s(1)-cx);
       mg=(g(2)-cy)/(g(1)-cx);
       ts=atan(ms); tg=atan(mg);
       if ts>tg
           theta(i)=-pi/3;
       else
           theta(i)=pi/3;
       end
       %rotazione da start verso goal, 
       
    end
    
    
    %ostacoli
    NJrXX=zeros(size(NJaXX));
    NJrYY=zeros(size(NJaXX));
    
    for i=1:size(O,2)
        in=inpolygon(XX,YY,Obs{i}(1,:),Obs{i}(2,:));
        tmpx=zeros(size(NJaXX));
        tmpy=zeros(size(NJaXX));
        for j=1:size(in,1)
            for k=1:size(in,2)
                if in(j,k)==0
                    continue;
                end
                tmpx=tmpx+NJrx(XX,YY,XX(j,k),YY(j,k)).*rho(XX,YY,XX(j,k),YY(j,k));
                tmpy=tmpy+NJry(XX,YY,XX(j,k),YY(j,k)).*rho(XX,YY,XX(j,k),YY(j,k));
            end
        end
        
        for j=1:size(NJrXX,1)
            for k=1:size(NJrXX,2)
                curr=[tmpx(j,k),tmpy(j,k)];
                curr=rm(theta(i))*curr';
                tmpx(j,k)=curr(1); tmpy(j,k)=curr(2);
            end
        end
        NJrXX=NJrXX+tmpx;
        NJrYY=NJrYY+tmpy;

    end
    
    %enviroment
    e=obstacleToShape(E,0.1);
    for i=1:size(e,1)
        NJrXX=NJrXX+NJrx(XX,YY,e(i,1),e(i,2)).*rho(XX,YY,e(i,1),e(i,2));
        NJrYY=NJrYY+NJry(XX,YY,e(i,1),e(i,2)).*rho(XX,YY,e(i,1),e(i,2)); 
    end
    
   
    
    

    
    %gradiente totale
    NJx=wa*NJaXX+wo*NJrXX;
    NJy=wa*NJaYY+wo*NJrYY;
    %normalizziamo il gradiente per avere vettori unitari, potendo così
    %descrivere la direzione
    NJxn=NJx./sqrt(NJx.^2+NJy.^2);
    NJyn=NJy./sqrt(NJx.^2+NJy.^2);

%%  Path Planning

    TH=dBot;
    iter=10000;
    X=zeros(iter,2);
    X(1,:)=[s(1),s(2)];

    for k=2:iter
        Xcur=X(k-1,:);
        
        %troviamo le coordinate più vicine al punto corrente nella meshgrid
        d=(Xcur(1)-XX).^2+(Xcur(2)-YY).^2;
        [~, ind] = min(d(:));
        
        x=XX(ind);
        y=YY(ind);
        i=find(xx==x);
        j=find(yy==y);
        
        
        %Richiamiamo le funzioni interp2 per ricavare il valore del
        %gradiente nel punto corrente
        grad_x = interp2(NJxn, i, j);
        grad_y = interp2(NJyn, i, j);
        grad = [grad_x, grad_y];

        
        %calcoliamo il prossimo punto spostandoci di distanza alpha verso
        %la direzione descritta dal gradiente
        Xsucc=Xcur-alpha*grad;
        
        X(k,:)=Xsucc;
        if norm(Xsucc-g)<=TH %condizione di stop sul goal
            break;
        end
    end
    via=X(1:k,:); 
    
    %plot
    figure('name','APF'); hold on;
    quiver(XX,YY,-NJxn,-NJyn); hold on;
    plot(s(1),s(2),'x'); hold on;
    plot(g(1),g(2),'O'); hold on;
    for i=1:size(O,2)
       plot(O{i}(:,1),O{i}(:,2),'-g','LineWidth',2);       hold on;
    end
    plot(e(:,1),e(:,2),'-r','LineWidth',2); hold on;
    plot(X(:,1),X(:,2),'xr');
    axis('equal');
    grid on;
    
     
end
