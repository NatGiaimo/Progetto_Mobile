function Xd = Linearization(t,botState,x_s,y_s,theta_s,xd_s,yd_s,xdd_s,ydd_s,a,delta)
%t istante di tempo nel quale stiamo effettuando l'operazione corrente
%x_s=x_start e xs=x_s(t) ...
x=botState(1); y=botState(2); theta=botState(3);

xs=x_s(t); ys=y_s(t); thetas=theta_s(t);
xds=xd_s(t); yds=yd_s(t);
xdds=xdd_s(t); ydds=ydd_s(t);

%abbiamo le componenti della velocità tangenziale (xd yd) possiamo quindi calcolarla
vs=sqrt(xds.^2+yds.^2);
%poiché il nostro modello è un uniciclo, w(t)=thetad(t). thetad è pari
%alla variazione dell'angolo compreso tra il vettore velocità e accellerazione.
%Svolgiamo il loro prodotto vettoriale e normalizziamo il risultato
%dividendolo per il quadrato della velocità
ws=(ydds*xds-yds*xdds)/(vs^2);

%possiamo adesso applicare la legge di controllo
k1=2*delta*a;
k2=(a.^2-ws.^2)/(vs);
k3=k1;

%calcoliamo gli errori
ex=xs-x; ey=ys-y; etheta=thetas-theta;

v=vs*cos(etheta)+k1*ex;
w=ws+k2*ey+k3*etheta;

%ricaviamo adesso le variabili per il modello uniciclo
%Xd=[xd;yd;thetad]
Xd=[v*cos(theta); v*sin(theta); w];

end

