function sol=TrajTracking(controller, planner, botState,v,tf)
%% ricavo funzioni tempo variabili
tspan=linspace(0,tf,size(v,1))';

if planner == 1
    %applichiamo un filtro sui dati ricevuti dal planner APF
    xfilt=medfilt1(v(:,1),30);
    yfilt=medfilt1(v(:,2),30);
    xfun=csapi(tspan,xfilt);
    yfun=csapi(tspan,yfilt);
else
    xfun=csapi(tspan,v(:,1));
    yfun=csapi(tspan,v(:,2));
end
%ricaviamo derivate prime e seconde di xfun e yfun
xdfun=fnder(xfun,1);
ydfun=fnder(yfun,1);
xddfun=fnder(xfun,2);
yddfun=fnder(yfun,2);

%creiamo delle funzioni in modo da garantire il funzionamento di ode45 nel
%tempo
x=@(t)(ppval(xfun,t));
y=@(t)(ppval(yfun,t));
xd=@(t)(ppval(xdfun,t));
yd=@(t)(ppval(ydfun,t));
xdd=@(t)(ppval(xddfun,t));
ydd=@(t)(ppval(yddfun,t));
theta=@(t)(atan2(yd(t),xd(t)));

%% dichiarazione legge di controllo
switch controller
    case 1
        %a>0; delta in (0,1)
        switch planner
            case 1
                a=1; delta=0.5;
            case 2
                a=1.56; delta=0.5;
            case 3
                a=1.3; delta=0.99;
            case 4
                a=1.7; delta=0.5;
            otherwise
        end
        
        fun=@(t,X)Linearization(t,X,x,y,theta,xd,yd,xdd,ydd,a,delta);
    case 2
        %k1>0; k2>0; k3>0;
        switch planner
            case 1
                k1=@(v,w)(ones(size(v)));
                k3=@(v,w)(ones(size(v)).*3.5);
                k2=4; 
            case 2
                k1=@(v,w)(ones(size(v)));
                k3=@(v,w)(ones(size(v)).*2);
                k2=15;  
            case 3
                k1=@(v,w)(ones(size(v)));
                k3=@(v,w)(ones(size(v)).*2);
                k2=20;
            case 4
                k1=@(v,w)(ones(size(v)));
                k3=@(v,w)(ones(size(v)).*4);
                k2=10;  
            otherwise
        end
       
        fun=@(t,X)NonLinear(t,X,x,y,theta,xd,yd,xdd,ydd,k1,k2,k3);
    case 3
        %kx>0 ky>0
        kx=1; ky=1;
        b=0.05;
        fun=@(t,X)IOLinearization(t,X,x,y,xd,yd,kx,ky,b);
    otherwise
        disp('Valore variabile "controller" non valido.');
        return;
end
%% risoluzione ode
%richiamiamo ode45 per risolvere il sistema di equazioni differenziali
%fornito dalla tecnica di controllo scelta
[teval,sol]=ode45(fun,[0,tf],botState);

fPlot(xfun,yfun,xdfun,ydfun,xddfun,yddfun,sol,theta,teval);
end
function []=fPlot(x,y,xd,yd,xdd,ydd,sol,tfun,t)
xx = ppval(x, t);
yy = ppval(y, t);
xxd = ppval(xd, t);
yyd = ppval(yd, t);
xxdd = ppval(xdd, t);
yydd= ppval(ydd, t);
theta=feval(tfun, t);

figure('name','controller')

subplot(3,2,2)
title('Spline')
hold on
plot(t, xx);
hold on;
plot(t, yy);
hold on;
xlabel('Time');
ylabel('Value');
legend('X', 'Y');
grid on

subplot(3,2,4)
title('Spline*')
hold on
plot(t, xxd);
hold on;
plot(t, yyd);
hold on;
xlabel('Time');
ylabel('Value');
legend('Xd','Yd');
grid on

subplot(3,2,6)
title('Spline**')
hold on
plot(t, xxdd);
hold on;
plot(t, yydd);
xlabel('Time');
ylabel('Value');
legend('Xdd','Ydd');
grid on

subplot(3,2,[1,3,5])
title('Spline and ODE output')
hold on
plot(xx,yy,'-r')
hold on
plot(sol(:,1),sol(:,2),'--b')
hold on
legend('Spline','ODE output')
grid on

figure('name','errore')
subplot(3,1,1)
title('Errore x'); hold on;
plot(t, xx-sol(:,1)); hold on;
yline(0);
subplot(3,1,2)
title('Errore y'); hold on;
plot(t, yy-sol(:,2)); hold on;
yline(0);
subplot(3,1,3)
title('Errore theta'); hold on;
plot(t, theta-sol(:,3)); hold on;
yline(0);

end
