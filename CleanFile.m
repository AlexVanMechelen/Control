clearvars; clc; close all
PATH = pwd;
addpath("matlab_tools")
% set(gca,'YTick',-2*pi:pi/2:2*pi)
% set(gca,'YTickLabel',{'$-2\pi$','$-3\pi$/2','$-\pi$','$-\pi$/2','0','$\pi$/2','$\pi$','3$\pi$/2','2$\pi$'})
%% Opmaak
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
set(groot,'defaultLineMarkerSize',35)
set(groot,'defaultAxesFontSize',40)
get(0,'ScreenSize');
set(groot,'defaultFigurePosition',[0 0 ans(3) ans(4)])
set(groot,'defaultLineLineWidth',2)
set(groot,'defaultLegendLocation','best')
set(groot,'defaultAxesGridAlpha',0.5)
set(groot,'defaultAxesLineWidth',1)
set(groot,'defaultConstantLineLineWidth',2)
LW = 1; % LineWidth
%% Data
L = 0.3;
M = 0.5;
m = 0.2;
b = 0.1;
max_F = 10;
rail_length = 1;
cart_length = 0.2;
I = 0.006;
g = 9.81;
%% Matrices opstellen
N = I*(M+m) + M*m*L^2;
A = [0 1 0 0;
    0 -(I+m*L^2)*b/N m^2*g*L^2/N 0;
    0 0 0 1;
    0 -m*L*b/N m*g*L*(M+m)/N 0];
B = [0;
    (I+m*L^2)/N;
    0;
    m*L/N];
C = [1 0 0 0;
    0 0 1 0];
D = [0;
    0];
states = {'x' 'v' 'theta' 'w'};
inputs = {'u'};
outputs = {'x'; 'theta'};
S = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
Scomb = ss(A,[B B],C,0,'statename',states,'inputname',{'u1','u2'},'outputname',outputs);
%% Modes
OPEN_LOOP   = 0;
CLASSICAL_ANG   = 1;
CLASSICAL_COMB   = 2;
OBSERVER_TEST = 3;
STATE_SPACE = 4;
EXTENDED    = 5;
%% Startup Python
if ispc % Check for Windows OS
    system('closeSessions.bat');
    pause(1)
    system('start cmd /k "title SYSTEM & python system.py"');
    system('start cmd /k "title CONTROLLER & python controller.py"');
end
%% Discreet
Ts = 0.05;
Sd = c2d(S,Ts);
Sdtf = tf(Sd);
%% Plot Systeem open
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 60);
n_samples = 1/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = OPEN_LOOP;
w = 1;
set_mode_params(arduino, mode, w, [])
reset_system(arduino)
Y = get_response(arduino, w, n_samples);
x = Y(1,:); theta = Y(2,:);
close_connection(arduino)
clear arduino
%%
t = 0:Ts:1;
[y] = lsim(Sd,[ones(1,length(t))],t,[0 0 0 0]);
for i = 1:length(y(:,2))
    if y(i,2) > pi
        y(i:end,2) = y(i:end,2)-2*pi;
    elseif y(i,2) < -pi
        y(i:end,2) = y(i:end,2)+2*pi;
    end
end
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(t,y(:,1),'b-','HandleVisibility','off')
plot(t,x,'b--','HandleVisibility','off')
ylabel('Positie [m]')
ylim([0 4])
yyaxis right
plot(t,y(:,2),'r-','HandleVisibility','off')
plot(t,theta,'r--','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'w')
ylim([-pi-0.1 pi+0.1])
ylabel('Hoek [rad]')
set(gca,'YTick',-2*pi:pi/2:2*pi)
set(gca,'YTickLabel',{'$-2\pi$','$-3\pi$/2','$-\pi$','$-\pi$/2','0','$\pi$/2','$\pi$','3$\pi$/2','2$\pi$'})
xlabel('Tijd [s]')
title('\textbf{Impulsrespons open lus}','FontSize',45)
legend('Simulatie','Meting',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','southwest')
%exportgraphics(F,PATH+"/Plots-Video/PID/ResponsOpenLus.png",'Resolution',300)
%% Hoekcontroller
ps2 = pole(Sdtf(2));
zs2 = zero(Sdtf(2));
Rd2 = zpk([ps2(2:end)],[0,1.04],48.1,Ts);
%% Plot Hoekcontroller gesloten
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 10^3);
n_samples = 5/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = CLASSICAL_ANG;
w = 0.0;
[~,G2] = zero(Rd2);
set_mode_params(arduino, mode, w, cat(1, G2, cat(1, zero(Rd2), pole(Rd2))));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
x = Y(1,:); theta = Y(2,:);
close_connection(arduino)
clear arduino
%%
t = 0:Ts:5;
[y] = lsim(feedback(Rd2*Sd,[0 1]),[zeros(1,length(t))],t,[0 0 0.1 0 0 0]);
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(t,y(:,1),'b-','HandleVisibility','off')
plot(t,x,'b--','HandleVisibility','off')
ylabel('Positie [m]')
yyaxis right
plot(t,y(:,2),'r-','HandleVisibility','off')
plot(t,theta,'r--','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'w')
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{Impulsrespons gesloten lus met $R_{theta}$}','FontSize',45)
legend('Simulatie',"Meting",newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0.1\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Orientation','horizontal')
%exportgraphics(F,PATH+"/Plots-Video/PID/ImpulsResponsGeslotenLusR2.png",'Resolution',300)
%% Positiecontroller
ps1 = pole(Sdtf(1));
zs1 = zero(Sdtf(1));
Rd1 = zpk([ps1(1),1.3,ps1(3)],[0.2,0.75,zs1(3),0.07],2.06,Ts);
%% Plot Positiecontroller gesloten
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^2);
n_samples = 20/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = CLASSICAL_COMB;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 1;
set_mode_params(arduino, mode, 0, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2))));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
x = Y(1,:); theta = Y(2,:);
close_connection(arduino)
clear arduino
%%
t = 0:Ts:20;
[y] = lsim(minreal(feedback(Rd1*Sd,[1 0])),[ones(1,length(t))],t);
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(t,y(:,1),'b-','HandleVisibility','off')
plot(t,x,'b--','HandleVisibility','off')
ylabel('Positie [m]')
yyaxis right
plot(t,y(:,2),'r-','HandleVisibility','off')
plot(t,theta,'r--','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'w')
set(gca,'YTick',-2*pi:pi/2:2*pi)
set(gca,'YTickLabel',{'$-2\pi$','$-3\pi$/2','$-\pi$','$-\pi$/2','0','$\pi$/2','$\pi$','3$\pi$/2','2$\pi$'})
ylim([-3*pi/2 3*pi/2])
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{Impulsrespons gesloten lus met $R_{x}$}','FontSize',45)
legend('Simulatie','Meting',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','south','Orientation','horizontal')
%exportgraphics(F,PATH+"/Plots-Video/PID/StapresponsGeslotenLusR1.png",'Resolution',300)
%% State Observer
ps_d1 = [0,0,0.01,0.01];
L1 = place(Sd.A', Sd.C', ps_d1);
L1 = L1';
ps_d2 = [0.7,0.8,zs1(3),zs1(3)];
L2 = place(Sd.A', Sd.C', ps_d2);
L2 = L2';
observer1 = ss(Sd.A - L1*Sd.C, [Sd.B, L1], eye(4,4), 0, Ts);
observer2 = ss(Sd.A - L2*Sd.C, [Sd.B, L2], eye(4,4), 0, Ts);
%% Simulatie State Observer
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 30/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = OBSERVER_TEST;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2)),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
u = [Y(3,:);x;theta];
%% Plot State Observer
[y] = lsim(observer1,u,ts,[0 0 pi 0]);
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(ts,y(:,1),'b-','HandleVisibility','off')
plot(ts,x_hat,'b--','HandleVisibility','off')
plot(ts,real_x,'k:','HandleVisibility','off')
ylabel('Positie [m]')
yyaxis right
plot(ts,y(:,3),'r-','HandleVisibility','off')
plot(ts,theta_hat,'r--','HandleVisibility','off')
plot(ts,real_theta,'k:','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'k:')
plot(nan,nan,'w')
plot(nan,nan,'w')
set(gca,'YTick',-2*pi:pi:2*pi)
set(gca,'YTickLabel',{'$-2\pi$','$-\pi$','0','$\pi$','2$\pi$'})
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{snel}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$")
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverSnel.png",'Resolution',300)
%% Simulatie State Observer
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 30/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = OBSERVER_TEST;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2)),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
u = [Y(3,:);x;theta];
%% Plot State Observer
[y] = lsim(observer2,u,ts,[0 0 pi 0]);
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(ts,y(:,1),'b-','HandleVisibility','off')
plot(ts,x_hat,'b--','HandleVisibility','off')
plot(ts,real_x,'k:','HandleVisibility','off')
ylabel('Positie [m]')
yyaxis right
plot(ts,y(:,3),'r-','HandleVisibility','off')
plot(ts,theta_hat,'r--','HandleVisibility','off')
plot(ts,real_theta,'k:','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'k:')
plot(nan,nan,'w')
plot(nan,nan,'w')
set(gca,'YTick',-2*pi:pi:2*pi)
set(gca,'YTickLabel',{'$-2\pi$','$-\pi$','0','$\pi$','2$\pi$'})
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$")
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverTraag.png",'Resolution',300)
%% Simulatie gecombineerde observer
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 30/0.05+1;
ts = (0:n_samples-1)*Ts;
mode = OBSERVER_TEST;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2)),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
u = [Y(3,:);x;theta];
%% Plot State Observer positie
[y1] = lsim(observer1,u,ts,[0 0 pi 0]);
[y2] = lsim(observer2,u,ts,[0 0 pi 0]);
for i = 1:length(ts)
    if abs(theta(i)) > pi/3
        y(i,:) = y1(i,:);
    else
        y(i,:) = y2(i,:);
    end
end
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(ts,y(:,1),'b-','HandleVisibility','off')
plot(ts,x_hat,'b--','HandleVisibility','off')
plot(ts,real_x,'k:','HandleVisibility','off')
ylabel('Positie [m]')
yyaxis right
plot(ts,y(:,3),'r-','HandleVisibility','off')
plot(ts,theta_hat,'r--','HandleVisibility','off')
plot(ts,real_theta,'k:','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'k:')
plot(nan,nan,'w')
plot(nan,nan,'w')
set(gca,'YTick',-3*pi:pi:3*pi)
set(gca,'YTickLabel',{'$-3\pi$','$-2\pi$','$-\pi$','0','$\pi$','2$\pi$','$3\pi$'})
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$")
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombPA.png",'Resolution',300)
%%
F = figure;
hold on
colororder({'b','r'})
plot(ts,y(:,1),'b-','HandleVisibility','off')
plot(ts,x_hat,'r-','HandleVisibility','off','LineWidth',1)
plot(ts,real_x,'k:','HandleVisibility','off')
plot(nan,nan,'b-')
plot(nan,nan,'r')
plot(nan,nan,'k:')
plot(nan,nan,'w')
ylabel('Positie [m]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','northeast')
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombP.png",'Resolution',300)
%%
F = figure;
hold on
colororder({'b','r'})
plot(ts,y(:,3),'b-','HandleVisibility','off')
plot(ts,theta_hat,'r-','HandleVisibility','off','LineWidth',1)
plot(ts,real_theta,'k:','HandleVisibility','off')
plot(nan,nan,'b-')
plot(nan,nan,'r-')
plot(nan,nan,'k:')
plot(nan,nan,'w')
ylabel('Hoek [rad]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','northeast')
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombA.png",'Resolution',300)
%% Plot State Observer snelheid
F = figure;
hold on
colororder({'b','r'})
yyaxis left
plot(ts,y(:,2),'b-','HandleVisibility','off')
plot(ts,v_hat,'b--','HandleVisibility','off')
plot(ts,real_v,'k:','HandleVisibility','off')
ylabel('Snelheid [m/s]')
yyaxis right
plot(ts,y(:,4),'r-','HandleVisibility','off')
plot(ts,theta_dot_hat,'r--','HandleVisibility','off')
plot(ts,real_theta_dot,'k:','HandleVisibility','off')
plot(nan,nan,'k-')
plot(nan,nan,'k--')
plot(nan,nan,'k:')
plot(nan,nan,'w')
plot(nan,nan,'w')
ylabel('Hoeksnelheid [rad/s]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','northeast')
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombVR.png",'Resolution',300)
%%
F = figure;
hold on
colororder({'b','r'})
plot(ts,y(:,2),'b-','HandleVisibility','off')
plot(ts,v_hat,'r-','HandleVisibility','off','LineWidth',1)
plot(ts,real_v,'k:','HandleVisibility','off')
plot(nan,nan,'b-')
plot(nan,nan,'r')
plot(nan,nan,'k:')
plot(nan,nan,'w')
ylabel('Snelheid [m/s]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','northeast')
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombV.png",'Resolution',300)
%%
F = figure;
hold on
colororder({'b','r'})
plot(ts,y(:,4),'b-','HandleVisibility','off')
plot(ts,theta_dot_hat,'r-','HandleVisibility','off','LineWidth',1)
plot(ts,real_theta_dot,'k:','HandleVisibility','off')
plot(nan,nan,'b-')
plot(nan,nan,'r-')
plot(nan,nan,'k:')
plot(nan,nan,'w')
ylabel('Hoeksnelheid [rad/s]')
xlabel('Tijd [s]')
title('\textbf{$Observer_{traag}$}','FontSize',45)
legend('Simulatie','Meting','Werkelijke waarde',newline+"$x_0 = 0\ m$"+newline+"$v_0 = 0\ m/s$"+newline+"$\theta_0 = 0\ rad$"+newline+"$\omega_0 = 0\ rad/s$",'Location','northeast')
%exportgraphics(F,PATH+"/Plots-Video/Observer/ObserverCombR.png",'Resolution',300)

















%% Plots
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
Yso = Y;
t_obs1 = find(abs(theta)>pi/3);
%% Plots
xtop = x_hat;
xbot = x_hat;
xtop(~t_obs1) = nan;
xbot(t_obs1) = nan;

figure;
tiles = tiledlayout(2,2);
%subplot(2,2,1)
nexttile
hold on;
plot(ts, xtop,'r', ts, xbot, 'b','LineWidth',LW);ylabel('Positie [m]'); xlabel('t [s]');
plot(ts,real_x,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

thetatop = theta_hat;
thetabot = theta_hat;
thetatop(~t_obs1) = nan;
thetabot(t_obs1) = nan;

%subplot(2,2,2)
nexttile
hold on; 
plot(ts, thetatop,'r', ts, thetabot, 'b','LineWidth',LW);ylabel('Theta [rad]'); xlabel('t [s]');
plot(ts,real_theta,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

vtop = v_hat;
vbot = v_hat;
vtop(~t_obs1) = nan;
vbot(t_obs1) = nan;

%subplot(2,2,3)
nexttile
hold on; 
plot(ts, vtop,'r', ts, vbot, 'b','LineWidth',LW); ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts,real_v,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

theta_dottop = theta_dot_hat;
theta_dotbot = theta_dot_hat;
theta_dottop(~t_obs1) = nan;
theta_dotbot(t_obs1) = nan;

%subplot(2,2,4)
nexttile
hold on; 
plot(ts, theta_dottop,'r', ts, theta_dotbot, 'b','LineWidth',LW); ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts,real_theta_dot,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%%exportgraphics(tiles,PATH+"/Plots-Video/Observer.png",'Resolution',300)
%%
v_est = ( x(2:end)-x(1:end-1) )/Ts;
theta_dot_est = ( theta(2:end)-theta(1:end-1) )/Ts;

figure;
tiles = tiledlayout(2,2); 
nexttile
hold on;
plot(ts(2:end), v_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,2)
nexttile
hold on; 
plot(ts(2:end), theta_dot_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,3)
nexttile
hold on; 
plot(ts(2:end), v_est,'b','LineWidth',LW); 
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

%subplot(2,2,4)
nexttile
hold on; 
plot(ts(2:end), theta_dot_est,'b','LineWidth',LW); 
ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%%exportgraphics(tiles,PATH+"/Plots-Video/Observer_Derivative.png",'Resolution',300)
%% State Space Feedback
Q = diag([38,1,10000,0]);
R = 1;
[Kd,S,e] = dlqr(Sd.A,Sd.B,Q,R)
%% Simulatie SSF
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 2400;
ts = (0:n_samples-1)*Ts;
mode = STATE_SPACE;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1,reshape(Kd,[],1),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
%% Plots
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
Yssf = Y;
t_obs1 = find(abs(theta)>pi/3);
xtop = x_hat;
xbot = x_hat;
xtop(~t_obs1) = nan;
xbot(t_obs1) = nan;

figure;
tiles = tiledlayout(2,2);
%subplot(2,2,1)
nexttile
hold on;
plot(ts, xtop,'r', ts, xbot, 'b','LineWidth',LW);ylabel('Positie [m]'); xlabel('t [s]');
plot(ts,real_x,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')
ylim([-5 5])

thetatop = theta_hat;
thetabot = theta_hat;
thetatop(~t_obs1) = nan;
thetabot(t_obs1) = nan;

%subplot(2,2,2)
nexttile
hold on; 
plot(ts, thetatop,'r', ts, thetabot, 'b','LineWidth',LW);ylabel('Theta [rad]'); xlabel('t [s]');
plot(ts,real_theta,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

vtop = v_hat;
vbot = v_hat;
vtop(~t_obs1) = nan;
vbot(t_obs1) = nan;

%subplot(2,2,3)
nexttile
hold on; 
plot(ts, vtop,'r', ts, vbot, 'b','LineWidth',LW); ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts,real_v,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')
ylim([-10 10])

theta_dottop = theta_dot_hat;
theta_dotbot = theta_dot_hat;
theta_dottop(~t_obs1) = nan;
theta_dotbot(t_obs1) = nan;

%subplot(2,2,4)
nexttile
hold on; 
plot(ts, theta_dottop,'r', ts, theta_dotbot, 'b','LineWidth',LW); ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts,real_theta_dot,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/StateSpace.png",'Resolution',300)
%%
v_est = ( x(2:end)-x(1:end-1) )/Ts;
theta_dot_est = ( theta(2:end)-theta(1:end-1) )/Ts;

figure;
tiles = tiledlayout(2,2); 
nexttile
hold on;
plot(ts(2:end), v_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,2)
nexttile
hold on; 
plot(ts(2:end), theta_dot_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,3)
nexttile
hold on; 
plot(ts(2:end), v_est,'b','LineWidth',LW); 
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

%subplot(2,2,4)
nexttile
hold on; 
plot(ts(2:end), theta_dot_est,'b','LineWidth',LW); 
ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/StateSpace_Derivative.png",'Resolution',300)
%% ESSF Positie
AE = [Sd.A,zeros(4,1);Sd.C(1,:),[1]];
BE = [Sd.B zeros(4,1);Sd.D(1) -1];
CE = [Sd.C, zeros(2,1)];
DE = [Sd.D; 0];
BEu0 = BE(:,1);
Q = diag([1,0,100000,0,1]);
R = 1;
[KE,SE,eE] = dlqr(AE,BEu0,Q,R)
Kd = KE(1,1:4);
Ki = KE(1,5);
%% Simulatie ESSF Positie
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 20;
ts = (0:n_samples-1)*Ts;
mode = EXTENDED;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1,reshape(Kd,[],1),Ki,reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1),0));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
%% Plots
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
Yessf = Y;
t_obs1 = find(abs(theta)>pi/3);
xtop = x_hat;
xbot = x_hat;
xtop(~t_obs1) = nan;
xbot(t_obs1) = nan;

figure;
tiles = tiledlayout(2,2);
%subplot(2,2,1)
nexttile
hold on;
plot(ts, xtop,'r', ts, xbot, 'b','LineWidth',LW);ylabel('Positie [m]'); xlabel('t [s]');
plot(ts,real_x,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

thetatop = theta_hat;
thetabot = theta_hat;
thetatop(~t_obs1) = nan;
thetabot(t_obs1) = nan;

%subplot(2,2,2)
nexttile
hold on; 
plot(ts, thetatop,'r', ts, thetabot, 'b','LineWidth',LW);ylabel('Theta [rad]'); xlabel('t [s]');
plot(ts,real_theta,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

vtop = v_hat;
vbot = v_hat;
vtop(~t_obs1) = nan;
vbot(t_obs1) = nan;

%subplot(2,2,3)
nexttile
hold on; 
plot(ts, vtop,'r', ts, vbot, 'b','LineWidth',LW); ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts,real_v,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

theta_dottop = theta_dot_hat;
theta_dotbot = theta_dot_hat;
theta_dottop(~t_obs1) = nan;
theta_dotbot(t_obs1) = nan;

%subplot(2,2,4)
nexttile
hold on; 
plot(ts, theta_dottop,'r', ts, theta_dotbot, 'b','LineWidth',LW); ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts,real_theta_dot,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/ExtendedStateSpace.png",'Resolution',300)
%%
v_est = ( x(2:end)-x(1:end-1) )/Ts;
theta_dot_est = ( theta(2:end)-theta(1:end-1) )/Ts;

figure;
tiles = tiledlayout(2,2); 
nexttile
hold on;
plot(ts(2:end), v_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,2)
nexttile
hold on; 
plot(ts(2:end), theta_dot_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,3)
nexttile
hold on; 
plot(ts(2:end), v_est,'b','LineWidth',LW); 
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

%subplot(2,2,4)
nexttile
hold on; 
plot(ts(2:end), theta_dot_est,'b','LineWidth',LW); 
ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/ExtendedStateSpace_Derivative.png",'Resolution',300)
%% ESSF Positie Hoek
% AE = [Sd.A,zeros(4,2);Sd.C,diag([1 1])];
% BE = [Sd.B zeros(4,2);Sd.D diag([-1 -1])];
% CE = [Sd.C, zeros(2,2)];
% DE = [Sd.D; zeros(2,1)];
% BEu0 = BE(:,1);
% Q = diag([1,0,1,0,1,1]);
% R = 1;
% [KE,SE,eE] = dlqr(AE,BEu0,Q,R)
% Kd = KE(1,1:4);
% Ki = KE(1,5);
% %% Simulatie ESSF Positie Hoek
% arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
% n_samples = 20;
% ts = (0:n_samples-1)*Ts;
% mode = EXTENDED;
% [~,G1] = zero(Rd1);
% [~,G2] = zero(Rd2);
% w = 0;
% set_mode_params(arduino, mode, w, cat(1,reshape(Kd,[],1),Ki,reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
% reset_system(arduino);
% Y = get_response(arduino, w, n_samples);
% close_connection(arduino)
% clear arduino
%% ESSF PI positie
AE = [Sd.A,zeros(4,1);Sd.C(1,:),[1]];
BE = [Sd.B zeros(4,1);Sd.D(1) -1];
CE = [Sd.C, zeros(2,1)];
DE = [Sd.D; 0];
BEu0 = BE(:,1);
Q = diag([100,0,100000,0,10]);
R = 5;
[KE,SE,eE] = dlqr(AE,BEu0,Q,R)
Kd = KE(1,1:4);
Ki = KE(1,5);
z = tf('z',Ts);
RI = Ki / (z - 1);
sysd_cl = ss(Sd.A - Sd.B * Kd, Sd.B, Sd.C, 0, Ts);  % Evt hier Bdu0
sysE_cl = feedback(RI*sysd_cl, [1, 1]);
P_ESS_I = log(pole(sysE_cl))/Ts; % Slowest one has Im component
z_PI = real(P_ESS_I(3));
z_PI = (P_ESS_I(end));
Kp = Ki / (1 - z_PI);
Kcorr = Kd - Kp * Sd.C(1,:);
%% Simulatie ESSF Positie
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 24000;
ts = (0:n_samples-1)*Ts;
mode = EXTENDED;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, mode, w, cat(1,reshape(Kcorr,[],1),Ki,reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1),Kp)); % Laatste component kan Kp zijn indien start bij up-positie
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
%% Plots
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
Yessf = Y;
t_obs1 = find(abs(theta)>pi/3);
xtop = x_hat;
xbot = x_hat;
xtop(~t_obs1) = nan;
xbot(t_obs1) = nan;

figure;
tiles = tiledlayout(2,2);
%subplot(2,2,1)
nexttile
hold on;
plot(ts, xtop,'r', ts, xbot, 'b','LineWidth',LW);ylabel('Positie [m]'); xlabel('t [s]');
plot(ts,real_x,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')
ylim([-5 5])

thetatop = theta_hat;
thetabot = theta_hat;
thetatop(~t_obs1) = nan;
thetabot(t_obs1) = nan;

%subplot(2,2,2)
nexttile
hold on; 
plot(ts, thetatop,'r', ts, thetabot, 'b','LineWidth',LW);ylabel('Theta [rad]'); xlabel('t [s]');
plot(ts,real_theta,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

vtop = v_hat;
vbot = v_hat;
vtop(~t_obs1) = nan;
vbot(t_obs1) = nan;

%subplot(2,2,3)
nexttile
hold on; 
plot(ts, vtop,'r', ts, vbot, 'b','LineWidth',LW); ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts,real_v,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')
ylim([-10 10])

theta_dottop = theta_dot_hat;
theta_dotbot = theta_dot_hat;
theta_dottop(~t_obs1) = nan;
theta_dotbot(t_obs1) = nan;

%subplot(2,2,4)
nexttile
hold on; 
plot(ts, theta_dottop,'r', ts, theta_dotbot, 'b','LineWidth',LW); ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts,real_theta_dot,'g','LineWidth',LW)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/ExtendedStateSpace.png",'Resolution',300)
%%
v_est = ( x(2:end)-x(1:end-1) )/Ts;
theta_dot_est = ( theta(2:end)-theta(1:end-1) )/Ts;

figure;
tiles = tiledlayout(2,2); 
nexttile
hold on;
plot(ts(2:end), v_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,2)
nexttile
hold on; 
plot(ts(2:end), theta_dot_hat(2:end),'b','LineWidth',LW);
ylabel('Snelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Observer','Werkelijke waarde')

%subplot(2,2,3)
nexttile
hold on; 
plot(ts(2:end), v_est,'b','LineWidth',LW); 
ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts(2:end),real_v(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

%subplot(2,2,4)
nexttile
hold on; 
plot(ts(2:end), theta_dot_est,'b','LineWidth',LW); 
ylabel('Hoeksnelheid [rad/s]'); xlabel('t [s]');
plot(ts(2:end),real_theta_dot(2:end),'g','LineWidth',LW)
legend('Derivatieve observer','Werkelijke waarde')

title(tiles,'\textbf{Extended SSF PI}','interpreter','latex')
tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
%exportgraphics(tiles,PATH+"/Plots-Video/ExtendedStateSpace_Derivative.png",'Resolution',300)