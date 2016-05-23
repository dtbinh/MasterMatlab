close all;
clear;
% old = digits(20);
% load SILTestLog/143651_landFBWA/mra/Data;
load SILFBWA11Teste/160541_landFBWA/mra/Data;

rad2deg = 180/pi;
deg2rad = pi/180;

Hardware = false;

%% Defining the net position
NetPos = struct;
NetPos.lat = 63.62881880654979;
NetPos.lon = 9.72883113772907;
NetPos.height = 150;
NetPos.z = -3;
NetPos.heading = -114.5;

%% Extract dubinsPath
PathDubins = PlanSpecification.maneuvers{1,1}{1,1}.data{1,1};
WP3 = PlanSpecification.maneuvers{1,1}{2,1}.data{1,1};
WP2 = PlanSpecification.maneuvers{1,1}{3,1}.data{1,1};
WP1 = PlanSpecification.maneuvers{1,1}{4,1}.data{1,1};

[WP3X, WP3Y, WP3Z] = geodetic2ned(WP3.lat*rad2deg,WP3.lon*rad2deg,WP3.z,NetPos.lat,NetPos.lon,NetPos.height-NetPos.z,wgs84Ellipsoid);
[WP2X, WP2Y, WP2Z] = geodetic2ned(WP2.lat*rad2deg,WP2.lon*rad2deg,WP2.z,NetPos.lat,NetPos.lon,NetPos.height-NetPos.z,wgs84Ellipsoid);
[WP1X, WP1Y, WP1Z] = geodetic2ned(WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,NetPos.lat,NetPos.lon,NetPos.height-NetPos.z,wgs84Ellipsoid);

xDubin = zeros(1,length(PathDubins.points{1,1}));
yDubin = zeros(1,length(PathDubins.points{1,1}));
zDubin = zeros(1,length(PathDubins.points{1,1}));
DubinsLat = PathDubins.lat*rad2deg;
DubinsLon = PathDubins.lon*rad2deg;
DubinsHeight = PathDubins.z;
[DubinsX0,DubinsY0,DubinsZ0] = geodetic2ned(DubinsLat,DubinsLon,DubinsHeight,NetPos.lat,NetPos.lon,NetPos.height-NetPos.z,wgs84Ellipsoid);
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

%% Extract state information
if (Hardware)
    C = unique(GpsFixRtk.src_ent);

    for i=1:length(C)
        row = find(EntityInfo.id==C(i));
        if strcmp(EntityInfo.component(row,1:9),'Sensors.R')
            src_ent = C(i);
        end
    end

    sizeOfRtk = length(find(GpsFixRtk.src_ent==src_ent));
    sizeOfExternal = length(ExternalNavData.state);
    diff = 0.2;

    %% Extract Rtk
    Rtk = struct;
    Rtk.n = zeros(1,sizeOfRtk);
    Rtk.e = zeros(1,sizeOfRtk);
    Rtk.d = zeros(1,sizeOfRtk);
    Rtk.type = zeros(1,sizeOfRtk);
    Rtk.base_lat = zeros(1,sizeOfRtk);
    Rtk.base_lon = zeros(1,sizeOfRtk);
    Rtk.base_height = zeros(1,sizeOfRtk);
    Rtk.timestamp = zeros(1,sizeOfRtk);
    Rtk.timediff = zeros(1,sizeOfRtk-1);
    j = 1;
    for i=1:length(GpsFixRtk.n)
        if (GpsFixRtk.src_ent(i)==src_ent)
            Rtk.n(1,j) = GpsFixRtk.n(i);
            Rtk.e(1,j) = GpsFixRtk.e(i);
            Rtk.d(1,j) = GpsFixRtk.d(i);
            Rtk.timestamp(1,j) = GpsFixRtk.timestamp(i);
            Rtk.base_lon(1,j) = GpsFixRtk.base_lon(i)*180/pi;
            Rtk.base_lat(1,j) = GpsFixRtk.base_lat(i)*180/pi;
            Rtk.base_height(1,j) = GpsFixRtk.base_height(i);
            if strcmp(GpsFixRtk.type(i,1:2),'FI')
                Rtk.type(j) = 3;
            elseif strcmp(GpsFixRtk.type(i,1:2),'FI')
                Rtk.type(j) = 2;
            else
                Rtk.type(j) = 0;
            end
            j = j+1;
        end
    end
    for i=1:sizeOfRtk-1
        Rtk.timediff(i) = Rtk.timestamp(i+1)-Rtk.timestamp(i);
    end
    Rtk.timeN = timeseries(Rtk.n,Rtk.timestamp);
    Rtk.timeE = timeseries(Rtk.e,Rtk.timestamp);
    Rtk.timeD = timeseries(Rtk.d,Rtk.timestamp);

    %% Extract Navsource used in system
    m_NavSources = struct;
    % m_NavSources.mask = zeros(length(NavSources.mask),1);
    m_NavSources.maskValue = zeros(length(NavSources.mask),1);
    for i=1:length(NavSources.mask)
    %     [m_NavSources.mask(i,:),~] = strsplit(NavSources.mask(i,:),{'GNSS_RTK','|'},'CollapseDelimiters',false,'DelimiterType','RegularExpression');
          index = strfind(NavSources.mask(i,:),'GNSS_RTK');
          m_NavSources.mask= NavSources.mask(1,index:index+7);
        if (strcmp(m_NavSources.mask,'GNSS_RTK'))
            m_NavSources.maskValue(i,1) = 1;
        else
            m_NavSources.maskValue(i,1) = 0;
        end
    end
end

%% Extract estimatedState
Estimated = struct;

Estimated.x = zeros(1,length(EstimatedState.timestamp));
Estimated.y = zeros(1,length(EstimatedState.timestamp));
Estimated.z = zeros(1,length(EstimatedState.timestamp));
Estimated.base_lat = zeros(1,length(EstimatedState.timestamp));
Estimated.base_lon = zeros(1,length(EstimatedState.timestamp));
Estimated.base_height = zeros(1,length(EstimatedState.timestamp));
Estimated.timestamp = EstimatedState.timestamp;
Estimated.DisN = zeros(1,length(EstimatedState.timestamp));
Estimated.DisE = zeros(1,length(EstimatedState.timestamp));
Estimated.DisD = zeros(1,length(EstimatedState.timestamp));
Estimated.PathN = zeros(1,length(EstimatedState.timestamp));
Estimated.PathE = zeros(1,length(EstimatedState.timestamp));
Estimated.PathD = zeros(1,length(EstimatedState.timestamp));
for i=1:length(EstimatedState.timestamp)
    Estimated.x(i) = EstimatedState.x(i);
    Estimated.y(i) = EstimatedState.y(i);
    Estimated.z(i) = EstimatedState.z(i);
    Estimated.base_lat(i) = EstimatedState.lat(i)*rad2deg;
    Estimated.base_lon(i) = EstimatedState.lon(i)*rad2deg;
    Estimated.base_height(i) = EstimatedState.height(i);
    if (Hardware)
        [Estimated.DisN(i),Estimated.DisE(i),Estimated.DisD(i)] = displacement(Estimated.base_lat(i),Estimated.base_lon(i),Estimated.base_height(i),...
                                                                                Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),...
                                                                                Estimated.x(i),Estimated.y(i),Estimated.z(i));
    end
    [Estimated.PathN(i),Estimated.PathE(i),Estimated.PathD(i)] = displacement(Estimated.base_lat(i),Estimated.base_lon(i),Estimated.base_height(i),...
                                                                                NetPos.lat,NetPos.lon,NetPos.height-NetPos.z,...
                                                                                Estimated.x(i),Estimated.y(i),Estimated.z(i));
end

%% Extract path state

PathState = struct;

PathState.crossTrack = PathControlState.y;
PathState.timestamp = PathControlState.timestamp;
%% Find cross track error and along track distance in the lateral plane

alpha_k = atan2(PathY(2)-PathY(1),PathX(2)-PathX(1));
i = 1;
j = 1;
lengthPath = length(PathY);
lengthState = length(EstimatedState.timestamp);
alongTrack = zeros(1,length(EstimatedState.timestamp));
crossTrack = zeros(1,length(EstimatedState.timestamp));
first = true;

while (i<(lengthPath-1) && j<lengthState)
    alongTrack(j) = (Estimated.PathN(j)-PathX(i))*cos(alpha_k) + (Estimated.PathE(j)-PathY(i))*sin(alpha_k);
    crossTrack(j) = -(Estimated.PathN(j)-PathX(i))*sin(alpha_k) + (Estimated.PathE(j)-PathY(i))*cos(alpha_k);
    if (alongTrack(j) >= sqrt((PathX(i+1)-PathX(i))^2+(PathY(i+1)-PathY(i))^2))
        i = i+1;
        alpha_k = atan2(PathY(i+1)-PathY(i),PathX(i+1)-PathX(i));
    end
    j = j+1;
end

figure(1)
plot(yDubin,xDubin);
grid on;
figure(2);
plot3(yDubin,xDubin,-zDubin);
grid on;
figure(3);
plot(WP(2,:),WP(1,:));
grid on;
figure(4)
plot(-WP(3,:));
grid on;
figure(5)
plot3(PathY,PathX,-PathZ);
grid on;
figure(6)
plot(PathY,PathX);
hold on;
plot(Estimated.PathE,Estimated.PathN,'r')
grid on;
figure(7)
plot(alongTrack(1:j))
figure(8)
plot(crossTrack(1:j))
grid on;
figure(9)
plot(-PathZ)
figure(10)
plot(PathState.timestamp-PathState.timestamp(1),PathState.crossTrack)
figure(11)
plot(Estimated.timestamp-Estimated.timestamp(1),Estimated.base_height-Estimated.z);
hold on;
plot(DesiredZ.timestamp-DesiredZ.timestamp(1),DesiredZ.value,'r');