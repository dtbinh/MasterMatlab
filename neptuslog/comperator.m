
clc
close all;
clear;

rad2deg = 180/pi;
deg2rad = pi/180;

Hardware = true; % RTK was available


filename = 'Agdenes20161804\ntnu-hexa-003\20160418\102153_agdenes_refsim_square_RTK\mra\Data.mat';


state1 = extractRTKEXT(Hardware,filename);

figure(1)
subplot(4,1,1);
plot(state1.Rtk.timestamp-state1.Rtk.timestamp(1),state1.Rtk.n);
xlabel('Time [s]');
ylabel('North [m]');
hold on;
grid on;
plot(state1.External.timestamp-state1.External.timestamp(1),state1.External.DisN,'--r');
legend('RTK-GNSS','External navigation system');
subplot(4,1,2);
plot(state1.Rtk.timestamp-state1.Rtk.timestamp(1),state1.Rtk.e);
hold on;
grid on;
plot(state1.External.timestamp-state1.External.timestamp(1),state1.External.DisE,'--r');
xlabel('Time [s]')
ylabel('East [m]')
subplot(4,1,3);
plot(state1.Rtk.timestamp-state1.Rtk.timestamp(1),state1.Rtk.d);
hold on;
grid on;
plot(state1.External.timestamp-state1.External.timestamp(1),state1.External.DisD,'--r');
xlabel('Time [s]')
ylabel('Down [m]')
subplot(4,1,4);
plot(state1.Rtk.timestamp-state1.Rtk.timestamp(1),state1.Rtk.type);
legend('RTK-GNSS','UAV navigation source')
ylabel(gca,'RTKGNSS solution type');
set(gca,'Box','off','YTick',[0 2 3],'YTickLabel',{'NONE', 'FLOAT', 'FIX'});
figure(2)
subplot(3,1,1);
plot(state1.comp.x.Time-state1.comp.x.Time(1),state1.comp.x.Data(:));
grid on;
xlabel('Time [s]')
ylabel('Difference[m]')
title('North')
legend('Bias')
subplot(3,1,2);
plot(state1.comp.y.Time-state1.comp.y.Time(1),state1.comp.y.Data(:));
grid on;
xlabel('Time [s]')
title('East')
ylabel('Difference [m]')
subplot(3,1,3);
plot(state1.comp.z.Time-state1.comp.z.Time(1),state1.comp.z.Data(:));
grid on;
xlabel('Time [s]')
title('Down')
ylabel('Difference [m]')