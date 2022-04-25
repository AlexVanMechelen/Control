clear all, clc, close all
%% define parameters and create tcp/ip object
arduino = tcpclient('127.0.0.1', 6012, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.05;
n_samples = 200;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%%
mode = OPEN_LOOP;
w = -1.0;
set_mode_params(arduino, mode, w, [])
%input('press enter')
reset_system(arduino)

w = 1.0;
Y = get_response(arduino, w, n_samples);

%%
x = Y(1,:);, theta = Y(2,:);, u = Y(3,:);
figure; plot(ts, x); title("x"); xlabel('t')
figure; plot(ts, theta); title("theta"); xlabel('t')
figure; plot(ts, u); title("u"); xlabel('t')

%% Estimate parameters
% Niet van toepassing
% Ook nog geen idee hoe alle params te schatten
% Terug in pos plaatsen | versnelling geven | 
% Sys heeft aantal params -> alle params indiv nodig?
%  MOdelisatie opdat erna regeling -> TF opstellen en hoeveel onbekende
%  params om deze te schatten
p = polyfit(ts(1:43), x(1:43), 2);
x_fit = polyval(p, ts);

figure; hold on;
plot(ts, x, ts, x_fit);

% Root locus van een onstabiel sys met 4 polen
% Klassieke regelaar -> niet vanzelfsprekend

% Eerste doel = stabiliseren, dan regelen
% pzmap discrete systeem

% Regelaar om systeem stabiel te krijgen
% evt meerdere regelaars om dit stabiel te krijgen
% Er is een pool en een onstabiele -> men kan beide niet cancellen
% Je moet altijd een segment rechts hebben en die segmenten gaan toch nog
% binnen de eenheidscirkel moeten komen
% Daarrond nog een 2de regelaar toevoegen om een pool toe te voegen in 1 -> polen
% verde laten evolueren en blijven in de eenheidscirkel
% In regelaar ook een pool in 0 te krijgen  (stabiliseren en ppol toevoegen
% -> moeilijk voor 1 regelaar

%%
mode = CLASSICAL;
w = 0.0;
set_mode_params(arduino, mode, w, cat(1, 48.1, cat(1, zero(Rd), pole(Rd))));

%input('press enter')

w = 0.1;
reset_system(arduino);
Y = get_response(arduino, w, n_samples);
x = Y(1,:);, theta = Y(2,:);, u=Y(3,:);
figure; plot(ts, x); title("x"); xlabel('t')
figure; plot(ts, theta); title("theta"); xlabel('t')
figure; plot(ts, u); title("u"); xlabel('t')

%%
close_connection(arduino)
clear arduino
