%Giaimo Natale, Matricola 209424
clear;
close all;
clc;
addpath('PathPlanning');
addpath('TrajectoryTracking')
%% start
%1=APF; 2=DPF; 3=VoronoiDiagrams; 4=VisbilityGraphs
planner=2;
%1=Linearization; 2=NonLinear; 3=IOLinearization
controller=3;
tFinal=30; %tempo nel quale il robot deve raggiungere il goal

dBot=0.2; %dimensione laterale del robot [m]    
[O,E]=obstacles(dBot);

s=[1,0.5];  %start
g=[4.5,0.5];    %goal
botState=[0.5;0.5;pi]; %stato iniziale del robot [x,y,theta]
%% path planning
via=[];
switch planner
    case 1
        via=APF(E,O,s,g,dBot);
    case 2
        via=DPF(E,O,s,g);
    case 3
        via=VoronoiDiag(E,O,s,g);
    case 4
        via=VisibilityGraph(E,O,s,g);
    otherwise
        disp('Valore della variabile "planner" non valido');
        return;
end
    
%% trajectory tracking
sol=TrajTracking(controller,planner,botState,via,tFinal);

%% plot finali
plotEnviroment(O,E,s,g);
plot(via(:,1),via(:,2),'xr');
hold on
plot(sol(:,1),sol(:,2),'-b','LineWidth',2);
legend('PathPlanning output','TrajectoryTracking output');
%% FUNZIONI

function []=plotEnviroment(O,E,start,goal)
    figure('name','Ambiente')
    plot([E(1,:),E(1,1)],[E(2,:),E(2,1)],'-');
    hold on
    for j=1:size(O,1)
        plot([O{j}(1,:),O{j}(1,1)],[O{j}(2,:),O{j}(2,1)]);
        hold on;
    end
    plot(start(1),start(2),'O');
    hold on;
    plot(goal(1),goal(2),'O');
    hold on;
    grid on;
    axis equal;
end
