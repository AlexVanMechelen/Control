set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
set(groot,'defaultLineMarkerSize',35)
set(groot,'defaultAxesFontSize',40)
set(0,'units','pixels');
ans = get(0,'ScreenSize');
set(groot,'defaultFigurePosition',[0 0 ans(3) ans(4)])
set(groot,'defaultLineLineWidth',2)
set(groot,'defaultLegendLocation','best')
set(groot,'defaultAxesGridAlpha',0.5)
set(groot,'defaultAxesLineWidth',1)
set(groot,'defaultConstantLineLineWidth',2)
PATH = pwd;
%% Relative error
F = figure;
x = 0:0.0001*pi:5*pi/4;
plot(x,abs(x-sin(x))./sin(x),x,abs(1-cos(x))./cos(x));
xlim([0 5*pi/4])
k = 10^2
ylim([-k k])
xticks(0:pi/4:5*pi/4)
xticklabels({'0','$\pi$/4','$\pi$/2','3$\pi$/4','$\pi$', '5$\pi$/4'})
title("Relative error small angle hypothesis")
ylabel("Relative error [-]")
xlabel("Angle [rad]")
exportgraphics(F,PATH+"/Plots-Video/SmallAngle/Small_angle_rel_err.png",'Resolution',300)
%% Absolute error
F = figure;
x = 0:0.0001*pi:5*pi/4;
plot(x,abs(x-sin(x)),x,abs(1-cos(x)));
xlim([0 5*pi/4])
ylim([0 5])
xticks(0:pi/4:5*pi/4)
xticklabels({'0','$\pi$/4','$\pi$/2','3$\pi$/4','$\pi$', '5$\pi$/4'})
title("Absolute error small angle hypothesis")
ylabel("Absolute error [-]")
xlabel("Angle [rad]")
exportgraphics(F,PATH+"/Plots-Video/SmallAngle/Small_angle_abs_err.png",'Resolution',300)