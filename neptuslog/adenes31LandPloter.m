clc
close all;
clear;

%% Plot a land path
load Agdenes2405/132859_land/mra/Data;

rad2deg = 180/pi;
deg2rad = pi/180;

Hardware = true; % RTK was unavailable

% [WP3X, WP3Y, WP3Z] = ecef2ned(WP3X,WP3Y,WP3Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP2X, WP2Y, WP2Z] = ecef2ned(WP2X,WP2Y,WP2Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP1X, WP1Y, WP1Z] = ecef2ned(WP1X,WP1Y,WP1Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);

%% 31 mai test 1
% filename = 'Agdenes_31mai/land/102726_landFBWA/mra/Data';
filename = 'Agdenes_31mai/land/103029_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/103344_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/105034_landFBWA/mra/Data';

%% 31 mai test 2
% filename = 'Agdenes_31mai/land/125420_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/125736_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/130234_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/130547_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/130911_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/131315_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/131844_landFBWA/mra/Data';
% filename = 'Agdenes_31mai/land/132137_landFBWA/mra/Data';

%% 1 juni 
% filename = 'Agdenes_1juni/glideslop8/081328_landFBWA/mra/data';
% filename = 'Agdenes_1juni/lookaheadraduis75/082345_landFBWALookahead30/mra/data';
% filename = 'Agdenes_1juni/segment10/082745_landFBWASegDistance10/mra/data';
% filename = 'Agdenes_1juni/segment10/083107_landFBWASegDistance10/mra/data';
% filename = 'Agdenes_1juni/segment10/083423_landFBWASegDistance10/mra/data';
% filename = 'Agdenes_1juni/glideslope7/083811_landFBWAglideangle7/mra/data';
% filename = 'Agdenes_1juni/finalapp90/084232_landFBWAfinalapp90/mra/data';
% filename = 'Agdenes_1juni/glideslope65/084656_landFBWAglideangle6k5/mra/data';

Path1 = pathExtract(filename);
state1 = stateExtract(Hardware,Path1,filename);
% plotFigures;

% filename =  'Agdenes2405/133227_land/mra/Data';
% % 
% % % Path2 = pathExtract(filename);
% state2 = stateExtract(Hardware,Path1,filename);
% % % plotFigures;
% % 
% filename = 'Agdenes2405/133625_land/mra/Data';
% % 
% % % Path3 = pathExtract(filename);
% state3 = stateExtract(Hardware,Path1,filename);
% % % plotFigures;
% % 
% filename = 'Agdenes2405/134029_land/mra/Data';
% % 
% % % Path4 = pathExtract(filename);
% state4 = stateExtract(Hardware,Path1,filename);
% % % plotFigures;

figure(1)
plot(Path1.PathY,Path1.PathX);
hold on;
grid on;
plot(Path1.PathY(1),Path1.PathX(1),'co');
plot(0,0,'rx');
plot(state1.Estimated.PathE,state1.Estimated.PathN,'b--');
% plot(state2.Estimated.PathE,state2.Estimated.PathN,'b--');
% plot(state3.Estimated.PathE,state3.Estimated.PathN,'b--');
% plot(state4.Estimated.PathE,state4.Estimated.PathN,'b--');
legend('Approach and landing path','Start position','Net position','X8 flight path');
xlabel('North [m]');
ylabel('East [m]');
figure(2);
plot3(Path1.PathY,Path1.PathX,-Path1.PathZ);
grid on;
hold on;
plot3(Path1.PathY(1),Path1.PathX(1),-Path1.PathZ(1),'co');
plot3(0,0,0,'rx');
plot3(state1.Estimated.PathE,state1.Estimated.PathN,-state1.Estimated.PathD,'b--');
legend('Approach and landing path','Start position','Net position','X8 flight path');
xlabel('North [m]');
ylabel('East [m]');
zlabel('Height from net [m]')
figure(3)
plot(state1.DesiredHeight.timestamp-state1.DesiredHeight.timestamp(1),state1.DesiredHeight.value,'r');
grid on;
hold on;
plot(state1.Estimated.timestamp-state1.Estimated.timestamp(1),state1.Estimated.base_height-state1.Estimated.z,'--b');
% plot(state2.Estimated.timestamp-state2.Estimated.timestamp(1),state2.Estimated.base_height-state2.Estimated.z,'-b');
% plot(state3.Estimated.timestamp-state3.Estimated.timestamp(1),state3.Estimated.base_height-state3.Estimated.z,'-b');
% plot(state4.Estimated.timestamp-state4.Estimated.timestamp(1),state4.Estimated.base_height-state4.Estimated.z,'-b');

ylabel('Height (WGS84) [m]');
xlabel('Time [s]');
legend('Desired height','X8 height');
figure(4)
plot(state1.PathState.timestamp-state1.PathState.timestamp(1),state1.PathState.crossTrack);
grid on;
ylim([-20 20]);
ylabel('Cross track error [m]');
xlabel('Time [s]');
figure(5)
plot(state1.PathState.timestamp-state1.PathState.timestamp(1),state1.PathState.alongtrack);
grid on;
figure(6)
plot(state1.DesiredPitch.timestamp-state1.DesiredPitch.timestamp(1),state1.DesiredPitch.value*rad2deg);
hold on;
plot(state1.Estimated.timestamp-state1.Estimated.timestamp(1),state1.Estimated.theta*rad2deg,'--r')
grid on;
figure(7)
plot(state1.DesiredRoll.timestamp-state1.DesiredRoll.timestamp(1),state1.DesiredRoll.value*rad2deg);
hold on;
grid on;
plot(state1.Estimated.timestamp-state1.Estimated.timestamp(1),state1.Estimated.phi*rad2deg,'--r');
% figure(4)
% plot(state1.Estimated.timestamp-state1.Estimated.timestamp(1),state1.crossTrack);
% figure(5)
% plot3(PathY,PathX,-PathZ);
% hold on;
% plot3(Estimated.PathE,Estimated.PathN,-Estimated.PathD);
% grid on;
% figure(6)
% plot(PathY,PathX);
% hold on;
% plot(Estimated.PathE,Estimated.PathN,'r')
% grid on;
% figure(7)
% plot(alongTrack(1:j))
% hold on;
% grid on;
% figure(8)
% plot(crossTrack(1:j))
% grid on;
% hold on;
% figure(9)
% plot(-PathZ)
% figure(10)
% plot(PathState.timestamp-PathState.timestamp(1),PathState.crossTrack)
% hold on;
% figure(11)
% plot(Estimated.timestamp-Estimated.timestamp(1),Estimated.base_height-Estimated.z);
% hold on;
% plot(DesiredZ.timestamp-DesiredZ.timestamp(1),DesiredZ.value,'r');
% grid on;