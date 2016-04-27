close all;
clear;
% old = digits(20);
load SILTestLog/143651_landFBWA/mra/Data;

rad2deg = 180/pi;


%% Extract dubinsPath
PathDubins = PlanSpecification.maneuvers{1,1}{1,1}.data{1,1};
WP3 = PlanSpecification.maneuvers{1,1}{2,1}.data{1,1};
WP2 = PlanSpecification.maneuvers{1,1}{3,1}.data{1,1};
WP1 = PlanSpecification.maneuvers{1,1}{4,1}.data{1,1};

[WP3X, WP3Y, WP3Z] = geodetic2ned(WP3.lat*rad2deg,WP3.lon*rad2deg,WP3.z,WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,wgs84Ellipsoid)
[WP2X, WP2Y, WP2Z] = geodetic2ned(WP2.lat*rad2deg,WP2.lon*rad2deg,WP2.z,WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,wgs84Ellipsoid)
[WP1X, WP1Y, WP1Z] = geodetic2ned(WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,wgs84Ellipsoid)

xDubin = zeros(1,length(PathDubins.points{1,1}));
yDubin = zeros(1,length(PathDubins.points{1,1}));
zDubin = zeros(1,length(PathDubins.points{1,1}));
DubinsLat = PathDubins.lat*rad2deg
DubinsLon = PathDubins.lon*rad2deg
DubinsHeight = PathDubins.z
[DubinsX0,DubinsY0,DubinsZ0] = geodetic2ned(DubinsLat,DubinsLon,DubinsHeight,WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,wgs84Ellipsoid);
xDubinECEF = zeros(1,length(PathDubins.points{1,1}));
yDubinECEF = zeros(1,length(PathDubins.points{1,1}));
zDubinECEF = zeros(1,length(PathDubins.points{1,1}));


for i=1:length(PathDubins.points{1,1})
    xDubin(1,i) = PathDubins.points{1,1}{i,1}.x + DubinsX0;
    yDubin(1,i) = PathDubins.points{1,1}{i,1}.y + DubinsY0;
    zDubin(1,i) = -PathDubins.points{1,1}{i,1}.z + DubinsZ0;
%     [xDubinECEF(1,i), yDubinECEF(1,i), zDubinECEF(1,i)] = ned2ecef(xDubin(1,i),yDubin(1,i),-zDubin(1,i),PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
end



% [WP3X, WP3Y, WP3Z] = ecef2ned(WP3X,WP3Y,WP3Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP2X, WP2Y, WP2Z] = ecef2ned(WP2X,WP2Y,WP2Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP1X, WP1Y, WP1Z] = ecef2ned(WP1X,WP1Y,WP1Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);

WP = [WP3X WP3Y WP3Z;WP2X WP2Y WP2Z;WP1X WP1Y WP1Z]';


PathX = [xDubin WP3X WP2X WP1X];
PathY = [yDubin WP3Y WP2Y WP1Y];
PathZ = [zDubin WP3Z WP2Z WP1Z];


figure(1)
plot(yDubin,xDubin);

figure(2);
plot3(yDubin,xDubin,-zDubin);

figure(3);
plot(WP(2,:),WP(1,:));
figure(4)
plot(WP(3,:));
figure(5)
plot3(PathY,PathX,-PathZ);
figure(6)
plot(PathY,PathX);

% digits(old)