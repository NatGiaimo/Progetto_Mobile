%Giaimo Natale, Matricola 209424
clear;
close all;
clc;
%% INIZIALIZZAZIONE
%L

syms Q [1 3];
DH=[
    [0 pi/2 0 Q(1)];
    [0.9 0 0 Q(2)];
    [0.9 0 0 Q(3)]];

%P(i,j), sulle righe x y e z e sulle colonne il punto
global P;
%punti forniti nella traccia
P=[ [0.8; 0.8; 0.5] [1.2; 0.8; 0.5] [1.0; 1.2; 0.5]];
%T(i)=Ti
global T;
T=[0,13,26,40];
%inizializziamo l'intervallo temporale
dTime=0.5;
timeSpan=0:dTime:40;

%% TRIANGOLARE

PP=zeros(length(timeSpan),3);
VV=zeros(length(timeSpan),3);
QQ=zeros(length(timeSpan),3);
QQd=zeros(length(timeSpan),3);

for i=1:length(timeSpan)
    %posizione e velocità
    [PP(i,:),VV(i,:)]=pointVelocity(timeSpan(i));
    %valori giunto
    QQ(i,:)=Antropomorfo_Cin_Inv(DH(:,1),PP(i,:));
    %Jacobiano
    J=Jacobiano_Antropomorfo(DH(:,1),QQ(i,:));
    %risoluzione sistema Qd=invJ(q)*Pd; J\Pd risuta computazionalmente più
    %veloce e precisa di inv(J)*Pd
    QQd(i,:)=J\transpose(VV(i,:));
    
end

plotAll(0, timeSpan, PP, VV, QQ, QQd);

%% CIRCONFERENZA

[rt,raggio]=RTsulPiano(P);

global rt_inv;
rt_inv=inv(rt);
global THETA;
THETA=pointToTheta(rt);

PPc=zeros(length(timeSpan),3);
VVc=zeros(length(timeSpan),3);
QQc=zeros(length(timeSpan),3);
QQdc=zeros(length(timeSpan),3);

for i=1:length(timeSpan)
    %procedura analoga a quella adottata per il percorso triangolare
    [PPc(i,:),VVc(i,:)]=pointVelocityCircum(raggio,timeSpan(i));
    QQc(i,:)=Antropomorfo_Cin_Inv(DH(:,1),PPc(i,:));
    J=Jacobiano_Antropomorfo(DH(:,1),QQc(i,:));
    QQdc(i,:)=J\transpose(VVc(i,:));
end

plotAll(1, timeSpan, PPc, VVc, QQc, QQdc)

    
%% FUNZIONI

function [] = plotAll(progressive, timeSpan, PP, VV, QQ, QQd)
    if progressive == 0
        str = 'triangolare';
    else
        str = ' circonferenza';
    end
    figure('name',(strcat('percorso ',str)))
    hold on
    plot3(PP(:,1),PP(:,2),PP(:,3));
    hold on
    global P;
    plot3(0,0,0);
    hold on
    plot3(P(1,:),P(2,:),P(3,:),'O');
    hold on
    grid on

    figure('name',(strcat('velocità end-effector ',str)))
    hold on
    plot(timeSpan,VV(:,1),timeSpan,VV(:,2),timeSpan,VV(:,3))
    legend("Vx","Vy","Vz")
    hold on
    grid on
    
    figure('name',(strcat('variabili dei giunti',str)))
    hold on
    plot(timeSpan,QQ(:,1),timeSpan,QQ(:,2),timeSpan, QQ(:,3))
    legend("Q1","Q2","Q3")
    hold on
    grid on
    
    figure('name',(strcat('velocità giunti',' ',str)))
    hold on
    plot(timeSpan,QQd(:,1),timeSpan,QQd(:,2),timeSpan, QQd(:,3))
    legend("Q1","Q2","Q3")
    hold on
    grid on
end

function [i,ni] = index(time,type)
    
    %PRE: type={0,1}; 0 se siamo nel caso percoso minima distanza, 1 sulla
    %circonferenza
        
    global T P;
    
    %riconoscimento intervallo corrente==percorso che stiamo seguendo
    %scorriamo gli intervalli di tempo
    for var=1:length(T)-1
        %riconosciamo in quale intervallo ci troviamo, tali che t in [T1,T2)
        if (T(var)<=time) && (T(var+1)>time)
            i=var;
            break;
        %poichè T2 non è compreso, avremo un problema con l'ultimo estremo,
        %forziamo l'indice ad appartenere all'ultimo intervallo quando
        %t=Tfinal
        elseif time==T(length(T))
            i=length(T)-1;
            break;
        end
    end
    
    %gestione overflow sull'index solo se siamo nel caso del percorso a
    %minima distanza, per la circonferenza l'ultimo indice rappresenta
    %l'angolo di partenza
    if (type==0) && ( (i+1)>size(P,2) )
        ni=1;
    else
        ni=i+1;
    end
end

function f = sigma(time,i)
    global T;
    f=(time-T(i))/(T(i+1)-T(i));
end

function [point,velocity] = pointVelocity(time)
    
    global P T;
    
    [i,ni]=index(time,0);
    s=sigma(time,i);
    
    point=P(:,i)+lambda(s)*(P(:,ni)-P(:,i));
    
    vm=(P(:,ni)-P(:,i))/(T(ni)-T(i));
    
    velocity=vm*lambdad(s);
end

function f = pTob(P)
    global rt_inv;
    temp=rt_inv*[P;0;1];
    %tutti i valori della circonferenza giacciono sul piano con z=0 per
    %costruzione, possiamo forzare manualmente il valore.
    f=temp(1:3);
end

function theta=pointToTheta(rt)
    %PRE: P=[P1, P2, P3] sul sistema di base
    %convertiamo i punti dal sistema di base al sistema sul piano
    global P;
    Pp=zeros(3,3);
    for i=1:3
        temp=rt*[P(:,i);1];
        Pp(:,i)=temp(1:3);
    end
    
    theta=zeros(3,1);
    for i=1:3
       theta(i) = atan2(Pp(1,i),Pp(2,i));
        if theta(i)<0
           theta(i)=theta(i)+2*pi;
        end
    end
    theta(4)=theta(1);
    %accadrà sicuramente, durante la rotazione, che venga compiuta una
    %rivoluzione completa, andiamo a manipolare il valore dei theta per
    %tener conto di questa situazione
    
    flag=0; %rotazione oraria -> -2pi
    if theta(1)>theta(2) %rotazione anti oraria +2pi
        flag=1;
    end

    %sommiamo o sottraiamo 2pi in base alla rotazione e al valore
    %dell'entry precedente
    for i=2:4
        if (flag==0) && ( theta(i)<theta(i-1) )
            theta(i:length(theta))=theta(i:length(theta))+2*pi;
        elseif (flag==1) && ( theta(i)>theta(i-1) )
            theta(i:length(theta))=theta(i:length(theta))-2*pi;
        end
    end
end

function [point,velocity] = pointVelocityCircum(raggio,time)
    
    [i,ni]=index(time,1);
    global THETA;
    %imponiamo gli estremi dell'intervallo corrente
    theta1=THETA(i);
    theta2=THETA(ni);
    
    %applichiamo la formula per ricavare il punto corrente
    s=sigma(time,i);
    theta=theta1+lambda(s)*(theta2-theta1);    
    point=raggio*[cos(theta); sin(theta)];
    point=pTob(point)';
    
    %troviamo adesso la velocità
    thetad=lambdad(s)*(theta2-theta1);
    velocity=raggio*[-sin(theta); cos(theta)]*thetad;
    velocity=pTob(velocity)';
end




















