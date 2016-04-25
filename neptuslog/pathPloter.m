close all;
clear;

load SILTestLog/143651_landFBWA/mra/Data;


%% Extract dubinsPath
PathDubins = PlanSpecification.maneuvers{1,1}{1,1}.data{1,1};
WP3 = PlanSpecification.maneuvers{1,1}{2,1}.data{1,1};
WP2 = PlanSpecification.maneuvers{1,1}{3,1}.data{1,1};
WP1 = PlanSpecification.maneuvers{1,1}{4,1}.data{1,1};
xDubin = zeros(1,length(PathDubins.points{1,1}));
yDubin = zeros(1,length(PathDubins.points{1,1}));
zDubin = zeros(1,length(PathDubins.points{1,1}));
xDubinECEF = zeros(1,length(PathDubins.points{1,1}));
yDubinECEF = zeros(1,length(PathDubins.points{1,1}));
zDubinECEF = zeros(1,length(PathDubins.points{1,1}));


for i=1:length(PathDubins.points{1,1})
    xDubin(1,i) = PathDubins.points{1,1}{i,1}.x;
    yDubin(1,i) = PathDubins.points{1,1}{i,1}.y;
    zDubin(1,i) = PathDubins.points{1,1}{i,1}.z;
    [xDubinECEF(1,i), yDubinECEF(1,i), zDubinECEF(1,i)] = ned2ecef(xDubin(1,i),yDubin(1,i),-zDubin(1,i),PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
end

[WP3X, WP3Y, WP3Z] = geodetic2ned(WP3.lat,WP3.lon,WP3.z,WP1.lat,WP1.lon,WP1.z,wgs84Ellipsoid);
[WP2X, WP2Y, WP2Z] = geodetic2ned(WP2.lat,WP2.lon,WP2.z,WP1.lat,WP1.lon,WP1.z,wgs84Ellipsoid);
[WP1X, WP1Y, WP1Z] = geodetic2ned(WP1.lat,WP1.lon,WP1.z,WP1.lat,WP1.lon,WP1.z,wgs84Ellipsoid);

% [WP3X, WP3Y, WP3Z] = ecef2ned(WP3X,WP3Y,WP3Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP2X, WP2Y, WP2Z] = ecef2ned(WP2X,WP2Y,WP2Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);
% [WP1X, WP1Y, WP1Z] = ecef2ned(WP1X,WP1Y,WP1Z,PathDubins.lat,PathDubins.lon,PathDubins.z,wgs84Ellipsoid);

WP = [WP3X WP3Y WP3Z;WP2X WP2Y WP2Z;WP1X WP1Y WP1Z];


PathX = [xDubin WP3X WP2X WP1X];
PathY = [yDubin WP3Y WP2Y WP1Y];
PathZ = [-zDubin WP3Z WP2Z WP1Z];


figure(1)
plot(yDubin,xDubin);

figure(2);
plot3(yDubin,xDubin,PathDubins.z+zDubin);

figure(3);
plot(WP(2,:),WP(1,:));
figure(4)
plot(WP(3,:));
figure(5)
plot3(PathY,PathX,PathDubins.z-PathZ);
figure(6)
plot(PathY,PathX);