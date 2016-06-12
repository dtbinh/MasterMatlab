close all; clear all;
rad2deg = 180/pi;

%% Defining WP
WP1 = [0 0]';
WP3 = [25 100]';
WP4 = [100 120]';
WP2 = [100 80]';
t = 0:0.01:1;
WP = [WP1 WP2 WP3 WP4];
lengthPaths = 0;
%% Calculate the tangential vector

ax1 = WP2(1,1);
ay1 = WP2(2,1);
psi1 = atan2(ay1,ax1);

lengthPaths = lengthPaths(end)+t*sqrt(ax1^2+ay1^2);
psi = ones(1,length(t))*psi1;

% lengthPaths = [lengthPaths lengthPaths(end)+sqrt(ax1^2+ay1^2)];

ax2 = WP3(1,1)-WP2(1,1);
ay2 = WP3(2,1)-WP2(2,1);
psi2 = atan2(ay2,ax2);
% lengthPaths = [lengthPaths lengthPaths(end)+sqrt(ax2^2+ay2^2)];

lengthPaths = [lengthPaths lengthPaths(end)+t*sqrt(ax2^2+ay2^2)];
psi = [psi ones(1,length(t))*psi2];

ax3 = WP4(1,1)-WP3(1,1);
ay3 = WP4(2,1)-WP3(2,1);
psi3 = atan2(ay3,ax3);
% lengthPaths = [lengthPaths lengthPaths(end)+sqrt(ax3^2+ay3^2)];

% psi = [psi1*180/pi*ones(1,60) psi2*180/pi*ones(1,60) psi3*180/pi*ones(1,60)];

% psi = [psi1 psi1 psi2 psi3];
lengthPaths = [lengthPaths lengthPaths(end)+t*sqrt(ax3^2+ay3^2)];
psi = [psi ones(1,length(t))*psi3];


%% Ploting
figure(1)
plot(WP(2,:),WP(1,:))
grid on;
xlabel('East [m]');
ylabel('North [m]');
title('Straight line path');
legend('Path');

figure(2)
plot(lengthPaths,psi*rad2deg);
grid on;
ylabel('[deg]');
xlabel('Path length [m]');
legend('Path tangential');
