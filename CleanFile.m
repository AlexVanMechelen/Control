clearvars; clc; close all
PATH = pwd;
addpath("matlab_tools")
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
%% Modes
OPEN_LOOP   = 0;
CLASSICAL_ANG   = 1;
CLASSICAL_COMB   = 2;
OBSERVER_TEST = 3;
STATE_SPACE = 4;
EXTENDED    = 5;
%% Discreet
Ts = 0.05;
Sd = c2d(S,Ts);
Sdtf = tf(Sd);
%% Hoekcontroller
ps2 = pole(Sdtf(2));
zs2 = zero(Sdtf(2));
Rd2 = zpk([ps2(2:end)],[0,1.04],48.1,Ts);
%% Positiecontroller
ps1 = pole(Sdtf(1));
zs1 = zero(Sdtf(1));
Rd1 = zpk([ps1(1),1.3,ps1(3)],[0.2,0.75,zs1(3),0.07],2.06,Ts);
%% State Observer
ps_d1 = [0,0,0.01,0.01];
L1 = place(Sd.A', Sd.C', ps_d1);
L1 = L1';
ps_d2 = [0.7,0.8,zs1(3),zs1(3)];
L2 = place(Sd.A', Sd.C', ps_d2);
L2 = L2';
%% Simulatie
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 1000;
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
%% Plots
x = Y(1,:); theta = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);

t_obs1 = find(abs(theta)>pi/3);
%% Opmaak
% set(groot,'defaulttextinterpreter','latex');
% set(groot, 'defaultAxesTickLabelInterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');
% set(groot,'defaultAxesXGrid','on')
% set(groot,'defaultAxesYGrid','on')
% set(groot,'defaultLineMarkerSize',35)
% set(groot,'defaultAxesFontSize',20)
% set(groot,'defaultFigurePosition',[0 0 1800 900])
% set(groot,'defaultLineLineWidth',2)
% set(groot,'defaultLegendLocation','best')
% set(groot,'defaultAxesGridAlpha',0.5)
% set(groot,'defaultAxesLineWidth',1)
% set(groot,'defaultConstantLineLineWidth',2)
%%
xtop = x_hat;
xbot = x_hat;
xtop(~t_obs1) = nan;
xbot(t_obs1) = nan;

tiles = tiledlayout(2,2); 
%subplot(2,2,1)
nexttile
hold on;
plot(ts, xtop,'r', ts, xbot, 'b');ylabel('Positie [m]'); xlabel('t [s]');
plot(ts,real_x,'g','LineWidth',1.2)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

thetatop = theta_hat;
thetabot = theta_hat;
thetatop(~t_obs1) = nan;
thetabot(t_obs1) = nan;

%subplot(2,2,2)
nexttile
hold on; 
plot(ts, thetatop,'r', ts, thetabot, 'b');ylabel('Theta [rad]'); xlabel('t [s]');
plot(ts,real_theta,'g','LineWidth',1.2)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

vtop = v_hat;
vbot = v_hat;
vtop(~t_obs1) = nan;
vbot(t_obs1) = nan;

%subplot(2,2,3)
nexttile
hold on; 
plot(ts, vtop,'r', ts, vbot, 'b'); ylabel('Snelheid [m/s]'); xlabel('t [s]');
plot(ts,real_v,'g','LineWidth',1.2)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

theta_dottop = theta_dot_hat;
theta_dotbot = theta_dot_hat;
theta_dottop(~t_obs1) = nan;
theta_dotbot(t_obs1) = nan;

%subplot(2,2,4)
nexttile
hold on; 
plot(ts, theta_dottop,'r', ts, theta_dotbot, 'b'); ylabel('Snelheid [rad/s]'); xlabel('t [s]');
plot(ts,real_theta_dot,'g','LineWidth',1.2)
legend('Observer grote hoeken','Observer kleine hoeken','Werkelijke waarde')

tiles.TileSpacing = 'tight';
tiles.Padding = "tight";
exportgraphics(tiles,PATH+"/Plots-Video/Observer.png")