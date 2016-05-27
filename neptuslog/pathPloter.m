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

pathExtract;
stateExtract;
plotFigures;

load SILFBWA11Teste/160815_landFBWA/mra/Data;

pathExtract;
stateExtract;
plotFigures;

load SILFBWA11Teste/161047_landFBWA/mra/Data;

pathExtract;
stateExtract;
plotFigures;
