function Xd = NonLinear(t,botState,x_s,y_s,theta_s,xd_s,yd_s,xdd_s,ydd_s,k1,k2,k3)
%t istante di tempo nel quale stiamo effettuando l'operazione corrente
%x_s=x_start e xs=x_s(t) ..
x=botState(1); y=botState(2); theta=botState(3);

xs=x_s(t); ys=y_s(t); thetas=theta_s(t);
xds=xd_s(t); yds=yd_s(t);
xdds=xdd_s(t); ydds=ydd_s(t);

%procedura analoga a quella adottata in Linearization.m
vs=sqrt(xds.^2+yds.^2);
ws=(ydds*xds-yds*xdds)/(vs^2);

ex=xs-x; ey=ys-y; eth=thetas-theta;

u1=-k1(vs,ws)*ex;
u2=-k2*vs*(sin(eth)/eth)*ey-k3(vs,ws)*eth;

v=vs*cos(eth)-u1;
w=ws-u2;

Xd=[v*cos(theta); v*sin(theta); w];

end