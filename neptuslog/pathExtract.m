%% Defining the net position
function [Path] = pathExtract(filename)

load(filename)
Path = struct;
%% Net pos
load NetPosSIL2205


C = unique(DesiredZ.src_ent);

for i=1:length(C)
row = find(EntityInfo.id==C(i));
%     if strcmp(EntityInfo.component(row(1,:),1:21),'Control.UAV.Ardupilot')
%         src_ent = C(i);
%     end
EntityInfo.component(row(1,:),1:21)
    if strcmp(EntityInfo.component(row(1,:),1:21),'Control.Path.HeightGl')
        src_ent = C(i);
    end
end

sizeOfArdupilot = length(find(DesiredZ.src_ent==src_ent));

DesiredHeight = struct;
DesiredHeight.timestamp = zeros(1,sizeOfArdupilot);
DesiredHeight.value = zeros(1,sizeOfArdupilot);
j = 1;
for (i=1:length(DesiredZ.timestamp))
    if (DesiredZ.src_ent(i)==src_ent)
        DesiredHeight.timestamp(j) = DesiredZ.timestamp(i);
        DesiredHeight.value(j) = DesiredZ.value(i);
        j = j+1;
    end
end
Path.DesiredHeight = DesiredHeight;

rad2deg = 180/pi;
deg2rad = pi/180;
Path.NetPos = NetPos;

%% Extract dubinsPath
PathDubins = PlanSpecification.maneuvers{1,1}{1,1}.data{1,1};
WP3 = PlanSpecification.maneuvers{1,1}{2,1}.data{1,1};
WP2 = PlanSpecification.maneuvers{1,1}{3,1}.data{1,1};
WP1 = PlanSpecification.maneuvers{1,1}{4,1}.data{1,1};

[WP3X, WP3Y, WP3Z] = geodetic2ned(WP3.lat*rad2deg,WP3.lon*rad2deg,WP3.z,Path.NetPos.lat,Path.NetPos.lon,Path.NetPos.height-Path.NetPos.z,wgs84Ellipsoid);
[WP2X, WP2Y, WP2Z] = geodetic2ned(WP2.lat*rad2deg,WP2.lon*rad2deg,WP2.z,Path.NetPos.lat,Path.NetPos.lon,Path.NetPos.height-Path.NetPos.z,wgs84Ellipsoid);
[WP1X, WP1Y, WP1Z] = geodetic2ned(WP1.lat*rad2deg,WP1.lon*rad2deg,WP1.z,Path.NetPos.lat,Path.NetPos.lon,Path.NetPos.height-Path.NetPos.z,wgs84Ellipsoid);

xDubin = zeros(1,length(PathDubins.points{1,1}));
yDubin = zeros(1,length(PathDubins.points{1,1}));
zDubin = zeros(1,length(PathDubins.points{1,1}));
DubinsLat = PathDubins.lat*rad2deg;
DubinsLon = PathDubins.lon*rad2deg;
DubinsHeight = PathDubins.z;
[DubinsX0,DubinsY0,DubinsZ0] = geodetic2ned(DubinsLat,DubinsLon,DubinsHeight,Path.NetPos.lat,Path.NetPos.lon,Path.NetPos.height-Path.NetPos.z,wgs84Ellipsoid);
for i=1:length(PathDubins.points{1,1})
    xDubin(1,i) = PathDubins.points{1,1}{i,1}.x + DubinsX0;
    yDubin(1,i) = PathDubins.points{1,1}{i,1}.y + DubinsY0;
    zDubin(1,i) = -PathDubins.points{1,1}{i,1}.z + DubinsZ0;
end

WP = [WP3X WP3Y WP3Z;WP2X WP2Y WP2Z;WP1X WP1Y WP1Z]';


PathX = [xDubin WP3X WP2X WP1X];
PathY = [yDubin WP3Y WP2Y WP1Y];
PathZ = [zDubin WP3Z WP2Z WP1Z];

Path.PathX = PathX;
Path.PathY = PathY;
Path.PathZ = PathZ;

Path.WP = WP;

end