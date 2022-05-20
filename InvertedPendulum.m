clearvars; clc; close all
PATH = pwd;
addpath("matlab_tools")
get(0,'ScreenSize');
set(groot,'defaultFigurePosition',[ans(3)/2 0 ans(3)/2 ans(4)])
%% Startup Python
if ispc % Check for Windows OS
    system('closeSessions.bat');
    pause(1)
    system('start cmd /k "title SYSTEM & python system.py"');
    system('start cmd /k "title CONTROLLER & python controller.py"');
    pause(3)
end
%% Data
L = 0.3;M = 0.5;m = 0.2;b = 0.1;I = 0.006;g = 9.81;
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
states = {'x' 'v' 'theta' 'w'};inputs = {'u'};outputs = {'x'; 'theta'};
S = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
%% Modes
OPEN_LOOP   = 0;CLASSICAL_ANG   = 1;CLASSICAL_COMB   = 2;
OBSERVER_TEST = 3;STATE_SPACE = 4;EXTENDED    = 5;
%% Discreet
Ts = 0.05;Sd = c2d(S,Ts);Sdtf = tf(Sd);
%% Plot Systeem open
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = OPEN_LOOP;
w = 1;
set_mode_params(arduino, mode, w, [])
reset_system(arduino)
Y = get_response(arduino, w, n_samples);
x = Y(1,:); theta = Y(2,:);
close_connection(arduino)
clear arduino
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x);title("Positie");hold on
nexttile;plot(ts,theta);title("Hoek");hold on
title(t,"Systeem open loop")
%% Hoekcontroller
ps2 = pole(Sdtf(2));zs2 = zero(Sdtf(2));
Rd2 = zpk([ps2(2:end)],[0,1.04],48.1,Ts);
%% Plot Hoekcontroller gesloten
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = CLASSICAL_ANG;
w = 0.0;
[~,G2] = zero(Rd2);
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, w, cat(1, G2, cat(1, zero(Rd2), pole(Rd2))));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
x = Y(1,:); theta = Y(2,:);
close_connection(arduino)
clear arduino
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x);title("Positie")
nexttile;plot(ts,theta);title("Hoek")
title(t,"Hoekcontroller")
%% Positiecontroller
ps1 = pole(Sdtf(1));zs1 = zero(Sdtf(1));
Rd1 = zpk([ps1(1),1.3,ps1(3)],[0.2,0.75,zs1(3),0.07],2.06,Ts);
%% Plot Positiecontroller + Hoek gesloten
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = CLASSICAL_COMB;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, 0, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2))));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
x1 = Y(1,:); theta1 = Y(2,:);
close_connection(arduino)
clear arduino
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x1);title("Positie")
nexttile;plot(ts,theta1);title("Hoek")
title(t,"Positie- en Hoekcontroller in parallel")
%% State Observer
ps_d1 = [0,0,0.01,0.01];L1 = place(Sd.A', Sd.C', ps_d1);L1 = L1';
ps_d2 = [0.7,0.8,zs1(3),zs1(3)];L2 = place(Sd.A', Sd.C', ps_d2);L2 = L2';
observer1 = ss(Sd.A - L1*Sd.C, [Sd.B, L1], eye(4,4), 0, Ts);
observer2 = ss(Sd.A - L2*Sd.C, [Sd.B, L2], eye(4,4), 0, Ts);
%% Simulatie State Observer
arduino = tcpclient('127.0.0.1', 6012, 'Timeout',60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = OBSERVER_TEST;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, w, cat(1, G1,cat(1,zero(Rd1),pole(Rd1)),G2, cat(1, zero(Rd2), pole(Rd2)),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x2 = Y(1,:); theta2 = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
figure(1);
t = tiledlayout(2,2);
nexttile;plot(ts,x2,ts,x_hat,ts,real_x);title("Positie");legend("Meting","Observer","Systeem")
nexttile;plot(ts,theta2,ts,theta_hat,ts,real_theta);title("Hoek");legend("Meting","Observer","Systeem")
nexttile;plot(ts,v_hat,ts,real_v);title("Snelheid");legend("Observer","Systeem")
nexttile;plot(ts,theta_dot_hat,ts,real_theta_dot);title("Hoeksnelheid");legend("Observer","Systeem")
title(t,"State Observer")
%% State Space Feedback
Q = diag([38,1,10000,0]);R = 1;
[Kd,S,e] = dlqr(Sd.A,Sd.B,Q,R);
%% Simulatie SSF
arduino = tcpclient('127.0.0.1', 6012, 'Timeout',60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = STATE_SPACE;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, w, cat(1,reshape(Kd,[],1),reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1)));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x3 = Y(1,:); theta3 = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x3);title("Positie")
nexttile;plot(ts,theta3);title("Hoek")
title(t,"State Space Feedback")
%% ESSF Positie
AE = [Sd.A,zeros(4,1);Sd.C(1,:),[1]];
BE = [Sd.B zeros(4,1);Sd.D(1) -1];
CE = [Sd.C, zeros(2,1)];
DE = [Sd.D; 0];
BEu0 = BE(:,1);
Q = diag([38,0,10000,0,10]);R = 1;
[KE,SE,eE] = dlqr(AE,BEu0,Q,R);
Kd = KE(1,1:4);Ki = KE(1,5);
%% Simulatie ESSF Positie
arduino = tcpclient('127.0.0.1', 6012, 'Timeout',60);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = EXTENDED;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, w, cat(1,reshape(Kd,[],1),Ki,reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1),0));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x4 = Y(1,:); theta4 = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x4);title("Positie")
nexttile;plot(ts,theta4);title("Hoek")
title(t,"Extended State Space Feedback Positie")
%% ESSFPI Positie
z = tf('z',Ts);
RI = Ki/(z-1);
sysd_cl = ss(Sd.A-Sd.B*Kd,Sd.B,Sd.C,0,Ts);
sysE_cl = feedback(RI*sysd_cl, [1, 1]);
P_ESS_I = log(pole(sysE_cl))/Ts;
z_PI = real(P_ESS_I(3));
z_PI = (P_ESS_I(end));
Kp = Ki/(1-z_PI);
Kcorr = Kd-Kp*Sd.C(1,:);
%% Simulatie ESSFPI Positie
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 2*10^3);
n_samples = 30/0.05+1;ts = (0:n_samples-1)*Ts;
mode = EXTENDED;
[~,G1] = zero(Rd1);
[~,G2] = zero(Rd2);
w = 0;
set_mode_params(arduino, OPEN_LOOP, 0, []);reset_system(arduino);pause(1)
set_mode_params(arduino, mode, w, cat(1,reshape(Kcorr,[],1),Ki,reshape(L1,[],1),reshape(Sd.A,[],1),reshape(Sd.B,[],1),reshape(Sd.C,[],1),reshape(L2,[],1),Kp));
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
close_connection(arduino)
clear arduino
x5 = Y(1,:); theta5 = Y(2,:); u = Y(3,:);
x_hat = Y(4,:); v_hat = Y(5,:); theta_hat = Y(6,:); theta_dot_hat = Y(7,:);
real_x = Y(8,:); real_v = Y(9,:); real_theta = Y(10,:); real_theta_dot = Y(11,:);
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x5);title("Positie")
nexttile;plot(ts,theta5);title("Hoek")
title(t,"Extended State Space Feedback PI Positie")
%% Comparison
pause(5)
figure(1);
t = tiledlayout(1,2);
nexttile;plot(ts,x1,ts,x2,ts,x3,ts,x4,ts,x5);title("Positie");legend("PID","PID+Observer","State Feedback","Extended State Feedback","Extended State Feedback PI")
nexttile;plot(ts,theta1,ts,theta2,ts,theta3,ts,theta4,ts,theta5);title("Hoek");legend("PID","PID+Observer","State Feedback","Extended State Feedback","Extended State Feedback PI")
title(t,"Vergelijking")