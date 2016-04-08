close all; clear all;


%% Defining WP
WP1 = [0 0]';
WP3 = [25 100]';
WP4 = [100 120]';
WP2 = [100 80]';

WP = [WP1 WP2 WP3 WP4];

%% Calculate the tangential vector

ax1 = WP2(1,1);
ay1 = WP2(2,1);
psi1 = atan2(ay1,ax1);

ax2 = WP3(1,1)-WP2(1,1);
ay2 = WP3(2,1)-WP2(2,1);
psi2 = atan2(ay2,ax2);

ax3 = WP4(1,1)-WP3(1,1);
ay3 = WP4(2,1)-WP3(2,1);
psi3 = atan2(ay3,ax3);

psi = [psi1*180/pi*ones(1,60) psi2*180/pi*ones(1,60) psi3*180/pi*ones(1,60)];

%% Ploting
figure(1)
plot(WP(2,:),WP(1,:))
grid on;
xlabel('East [m]');
ylabel('North [m]');
title('Straight line path');
legend('Path');

figure(2)
plot(psi);
grid on;
title('Tangential vector for straight line path');
% xlabel('Current way-point');
ylabel('\psi [degree]');
legend('Tangential vector');
% set(gca, 'XTick', [1 2 3 4 5 6 7 8 9 10]);
ax = gca;
ax.XTick = []
ax.XTickLabel = {'1','2','3'}