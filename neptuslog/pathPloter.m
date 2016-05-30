close all;
clear;

%% SIL simuleringer
% old = digits(20);
% load SILTestLog/143651_landFBWA/mra/Data;
load SILFBWA11Teste/160541_landFBWA/mra/Data;

rad2deg = 180/pi;
deg2rad = pi/180;

Hardware = false;

% [WP3X, WP3Y, WP3Z] = ecef2ned(WP3X,WP3Y,WP3Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP2X, WP2Y, WP2Z] = ecef2ned(WP2X,WP2Y,WP2Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP1X, WP1Y, WP1Z] = ecef2ned(WP1X,WP1Y,WP1Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);

filename = 'SILFBWA11Teste/160541_landFBWA/mra/Data';

Path1 = pathExtract(filename);
state1 = stateExtract(Hardware,Path1,filename);
% plotFigures;

filename =  'SILFBWA11Teste/160815_landFBWA/mra/Data';
% 
% % Path2 = pathExtract(filename);
state2 = stateExtract(Hardware,Path1,filename);
% % plotFigures;
% 
filename = 'SILFBWA11Teste/161047_landFBWA/mra/Data';
% 
% % Path3 = pathExtract(filename);
state3 = stateExtract(Hardware,Path1,filename);
% % plotFigures;
%
figure(1)
plot(Path1.PathY,Path1.PathX);
hold on;
grid on;
plot(Path1.PathY(1),Path1.PathX(1),'co');
plot(0,0,'rx');
% plot(state1.Estimated.PathE,state1.Estimated.PathN,'b--');
% plot(state2.Estimated.PathE,state2.Estimated.PathN,'b--');
% plot(state3.Estimated.PathE,state3.Estimated.PathN,'b--');
% legend('Approach and landing path','Start position','Net position','X8 flight path');
% xlabel('North [m]');
% ylabel('East [m]');
% figure(2);
% plot3(Path1.PathY,Path1.PathX,-Path1.PathZ);
% grid on;
% hold on;
% plot3(Path1.PathY(1),Path1.PathX(1),-Path1.PathZ(1),'co');
% plot3(0,0,0,'rx');
% plot3(state1.Estimated.PathE,state1.Estimated.PathN,-state1.Estimated.PathD,'b--');
% legend('Approach and landing path','Start position','Net position','X8 flight path');
% xlabel('North [m]');
% ylabel('East [m]');
% zlabel('Height from net [m]')
% figure(3)
% plot(Path1.DesiredHeight.timestamp-Path1.DesiredHeight.timestamp(1),Path1.DesiredHeight.value);
% grid on;
% hold on;
% plot(0,Path1.NetPos.height-Path1.PathZ(1),'co');
% plot(state1.Estimated.timestamp-state1.Estimated.timestamp(1),state1.Estimated.base_height-state1.Estimated.z,'-b');
% plot(state2.Estimated.timestamp-state2.Estimated.timestamp(1),state2.Estimated.base_height-state2.Estimated.z,'-b');
% plot(state3.Estimated.timestamp-state3.Estimated.timestamp(1),state3.Estimated.base_height-state3.Estimated.z,'-b');
% 
% ylabel('Height (WGS84) [m]');
% xlabel('Time [s]');
% figure(4)
% plot(state1.PathState.timestamp-state1.PathState.timestamp(1),state1.PathState.crossTrack);
% grid on;
% ylim([-20 20]);
% ylabel('Cross track error [m]');
% xlabel('Time [s]');