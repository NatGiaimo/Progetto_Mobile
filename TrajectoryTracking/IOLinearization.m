function Xd = IOLinearization(t,botState,x_s,y_s,xd_s,yd_s,kx,ky,b)
%t istante di tempo nel quale stiamo effettuando l'operazione corrente
%x_s=x_start e xs=x_s(t) ...
x=botState(1); y=botState(2); theta=botState(3);

xs=x_s(t); ys=y_s(t);
xb=x+b*cos(theta); yb=y+b*sin(theta);
xbds=xd_s(t); ybds=yd_s(t);

Ti=[ cos(theta),sin(theta); -sin(theta)/b, cos(theta)/b];

u1=xbds+(xs-xb).*kx;
u2=ybds+(ys-yb).*ky;

tmp=Ti*[u1;u2];
v=tmp(1); w=tmp(2);

Xd=[v*cos(theta); v*sin(theta); w];



end

