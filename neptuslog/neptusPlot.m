close all;
clear;

load 135749_gpsrtk_goto\mra\Data.mat;

C = unique(GpsFixRtk.src_ent);

for i=1:length(C)
    row = find(EntityInfo.id==C(i));
    if strcmp(EntityInfo.component(row,1:9),'Sensors.R')
        src_ent = C(i);
    end
end

sizeOfRtk = length(find(GpsFixRtk.src_ent==src_ent));
sizeOfExternal = length(ExternalNavData.state);


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
Rtk.timeN = timeseries(Rtk.n,Rtk.timestamp);
Rtk.timeE = timeseries(Rtk.e,Rtk.timestamp);
Rtk.timeD = timeseries(Rtk.d,Rtk.timestamp);

%% Extract External

External = struct;

External.x = zeros(1,sizeOfExternal);
External.y = zeros(1,sizeOfExternal);
External.z = zeros(1,sizeOfExternal);
External.base_lat = zeros(1,sizeOfExternal);
External.base_lon = zeros(1,sizeOfExternal);
External.base_height = zeros(1,sizeOfExternal);
External.timestamp = ExternalNavData.timestamp;

for i=1:sizeOfExternal
    External.x(i) = ExternalNavData.state{i,1}.x;
    External.y(i) = ExternalNavData.state{i,1}.y;
    External.z(i) = ExternalNavData.state{i,1}.z;
    External.base_lat(i) = ExternalNavData.state{i,1}.lat*180/pi;
    External.base_lon(i) = ExternalNavData.state{i,1}.lon*180/pi;
    External.base_height(i) = ExternalNavData.state{i,1}.height;
end
External.timeX = timeseries(External.x,External.timestamp);
External.timeY = timeseries(External.y,External.timestamp);
External.timeZ = timeseries(External.z,External.timestamp);

%% Sync timeseries
[External.timeX,Rtk.timeN] = synchronize(External.timeX,Rtk.timeN,'Union');
[External.timeY,Rtk.timeE] = synchronize(External.timeY,Rtk.timeE,'Union');
[External.timeZ,Rtk.timeD] = synchronize(External.timeZ,Rtk.timeD,'Union');

%% Find error
Error = struct;

Error.x = External.timeX-Rtk.timeN;
Error.y = External.timeY-Rtk.timeE;
Error.z = External.timeZ-Rtk.timeD;

%%
GpsEcef = struct;

GpsEcef.x = zeros(1,sizeOfRtk);
GpsEcef.y = zeros(1,sizeOfRtk);
GpsEcef.z = zeros(1,sizeOfRtk);
GpsEcef.timestamp = Rtk.timestamp;

[GpsEcef.x,GpsEcef.y,GpsEcef.z] = ned2ecef(Rtk.n,Rtk.e,Rtk.d,Rtk.base_lat,Rtk.base_lon,Rtk.base_height,wgs84Ellipsoid);

ExternalEcef = struct;

ExternalEcef.x = zeros(1,sizeOfExternal);
ExternalEcef.y = zeros(1,sizeOfExternal);
ExternalEcef.z = zeros(1,sizeOfExternal);
ExternalEcef.timestamp = External.timestamp;

[ExternalEcef.x,ExternalEcef.y,ExternalEcef.z] = ned2ecef(External.x,External.y,External.z,External.base_lat,External.base_lon,External.base_height,wgs84Ellipsoid);

%% Ned frame with rtk base as ref frame
GpsNed = struct;

GpsNed.n = zeros(1,sizeOfRtk);
GpsNed.e = zeros(1,sizeOfRtk);
GpsNed.d = zeros(1,sizeOfRtk);
GpsNed.timeN = zeros(1,sizeOfRtk);
GpsNed.timeE = zeros(1,sizeOfRtk);
GpsNed.timeD = zeros(1,sizeOfRtk);
GpsNed.timestamp = Rtk.timestamp;

[GpsNed.x,GpsNed.y,GpsNed.z] = ecef2ned(GpsEcef.x,GpsEcef.y,GpsEcef.z,Rtk.base_lat,Rtk.base_lon,Rtk.base_height,wgs84Ellipsoid);

GpsNed.timeN = timeseries(GpsNed.x,GpsNed.timestamp);
GpsNed.timeE = timeseries(GpsNed.y,GpsNed.timestamp);
GpsNed.timeD = timeseries(GpsNed.z,GpsNed.timestamp);

ExternalNed = struct;

ExternalNed.x = zeros(1,sizeOfExternal);
ExternalNed.y = zeros(1,sizeOfExternal);
ExternalNed.z = zeros(1,sizeOfExternal);
ExternalNed.timeX = zeros(1,sizeOfExternal);
ExternalNed.timeY = zeros(1,sizeOfExternal);
ExternalNed.timeZ = zeros(1,sizeOfExternal);
ExternalNed.timestamp = External.timestamp;

[ExternalNed.x,ExternalNed.y,ExternalNed.z] = ecef2ned(ExternalEcef.x,ExternalEcef.y,ExternalEcef.z,Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),wgs84Ellipsoid);

ExternalNed.timeX = timeseries(ExternalNed.x,ExternalNed.timestamp);
ExternalNed.timeY = timeseries(ExternalNed.y,ExternalNed.timestamp);
ExternalNed.timeZ = timeseries(ExternalNed.z,ExternalNed.timestamp);

[ExternalNed.timeX,GpsNed.timeN] = synchronize(ExternalNed.timeX,GpsNed.timeN,'Union');
[ExternalNed.timeY,GpsNed.timeE] = synchronize(ExternalNed.timeY,GpsNed.timeE,'Union');
[ExternalNed.timeZ,GpsNed.timeD] = synchronize(ExternalNed.timeZ,GpsNed.timeD,'Union');

Error = struct;

Error.x = ExternalNed.timeX-GpsNed.timeN;
Error.y = ExternalNed.timeY-GpsNed.timeE;
Error.z = ExternalNed.timeZ-GpsNed.timeD;

%% Figures
figure(1);
plot(EstimatedState.timestamp(:)-EstimatedState.timestamp(1),EstimatedState.height-EstimatedState.z);
hold on;
plot(ExternalNed.timestamp(:)-ExternalNed.timestamp(1),External.base_height-External.z,'g')
figure(2);
% plot(EstimatedState.y,EstimatedState.x);
plot(ExternalNed.y-2,ExternalNed.x-2);
hold on;
plot(Rtk.e(1,:),Rtk.n(1,:),'r');
% plot(External.y,External.x,'g');
plot(ExternalNed.y-mean(Error.y.Data(1,1,1:100)),ExternalNed.x-mean(Error.x.Data(1,1,1:100)),'c');
y = squeeze(ExternalNed.timeY.Data)-squeeze(Error.y.Data);
x = squeeze(ExternalNed.timeX.Data)-squeeze(Error.x.Data);
% plot(y,x,'k');
figure(3);
subplot(2,1,1);
plot(Error.x);
subplot(2,1,2);
plot(Error.y);
figure(4)
plot(ExternalNed.y-2,ExternalNed.x-2);
hold on;
plot(GpsNed.y,GpsNed.x,'r');
figure(5)
plot(Rtk.timestamp(:) - Rtk.timestamp(1), Rtk.type);
% ExternalNavData.state{2,1}