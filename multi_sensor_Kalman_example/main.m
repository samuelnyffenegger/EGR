%   EGR Rate estimation
%   Multi-sensor optimal information fusion Kalman filter example 
%   Samuel Nyffenegger
%   16.10.2017

%%  calculations

% initialize
clear all; close all; clc;
init();

% generate data
[x0_true, x_true, y, t] = generateData();

% centralized Kalman Filter
x_central = KF(y);




% print data
figure(1); clf;
    ax(1) = subplot(311); hold on; grid on;
        plot(t,x_true(1,:),'k','linewidth',1);
        plot(t,x_central(1,:),'b');
        ylabel('$s(t)$','Interpreter','latex')
        legend('x_{1,true}(t)','location','SE')
    ax(2) = subplot(312); hold on; grid on;
        plot(t,x_true(2,:),'k','linewidth',1);
        plot(t,x_central(2,:),'b');
        ylabel('$\dot{s}(t)$','Interpreter','latex')
        legend('x_{2,true}(t)','location','SE')
    ax(3) = subplot(313); hold on; grid on;
        plot(t,x_true(3,:),'k','linewidth',1);
        plot(t,x_central(3,:),'b');
        xlabel('$t$','Interpreter','latex');
        ylabel('$\ddot{s}(t)$','Interpreter','latex')
        legend('x_{3,true}(t)','location','SE')
    linkaxes(ax,'x');


