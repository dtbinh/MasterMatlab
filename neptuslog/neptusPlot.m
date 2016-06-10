close all;
clear;

% load 135749_gpsrtk_goto\mra\Data.mat;
% load Agdenes20161804\ntnu-hexa-003\20160418\092204_RTK-Test-Agdenes\mra\Data.mat


% TO be used in raport
load Agdenes20161804\ntnu-hexa-003\20160418\102153_agdenes_refsim_square_RTK\mra\Data.mat
% load Log11052016Agdenes\125025_agdenes_roll_tuning\mra\Data.mat
% Current log in use
% load ag20160525logs_hexaRTK/2hw/

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

%% Extract External

External = struct;

External.x = zeros(1,sizeOfExternal);
External.y = zeros(1,sizeOfExternal);
External.z = zeros(1,sizeOfExternal);
External.base_lat = zeros(1,sizeOfExternal);
External.base_lon = zeros(1,sizeOfExternal);
External.base_height = zeros(1,sizeOfExternal);
External.timestamp = ExternalNavData.timestamp;
External.DisN = zeros(1,length(ExternalNavData.timestamp));
External.DisE = zeros(1,length(ExternalNavData.timestamp));
External.DisD = zeros(1,length(ExternalNavData.timestamp));
for i=1:sizeOfExternal
    External.x(i) = ExternalNavData.state{i,1}.x;
    External.y(i) = ExternalNavData.state{i,1}.y;
    External.z(i) = ExternalNavData.state{i,1}.z;
    External.base_lat(i) = ExternalNavData.state{i,1}.lat*180/pi;
    External.base_lon(i) = ExternalNavData.state{i,1}.lon*180/pi;
    External.base_height(i) = ExternalNavData.state{i,1}.height;
    [External.DisN(i),External.DisE(i),External.DisD(i)] = displacement(External.base_lat(i),External.base_lon(i),External.base_height(i),Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),External.x(i),External.y(i),External.z(i));
end
External.timeX = timeseries(External.x,External.timestamp);
External.timeY = timeseries(External.y,External.timestamp);
External.timeZ = timeseries(External.z,External.timestamp);

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
for i=1:length(EstimatedState.timestamp)
    Estimated.x(i) = EstimatedState.x(i);
    Estimated.y(i) = EstimatedState.y(i);
    Estimated.z(i) = EstimatedState.z(i);
    Estimated.base_lat(i) = EstimatedState.lat(i)*180/pi;
    Estimated.base_lon(i) = EstimatedState.lon(i)*180/pi;
    Estimated.base_height(i) = EstimatedState.height(i);
    [Estimated.DisN(i),Estimated.DisE(i),Estimated.DisD(i)] = displacement(Estimated.base_lat(i),Estimated.base_lon(i),Estimated.base_height(i),Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),Estimated.x(i),Estimated.y(i),Estimated.z(i));
end


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
% subplot(2,1,1);
% plot3(Estimated.DisE,Estimated.DisN,Rtk.base_height(1)-Estimated.DisD,'b');
% grid on;
% hold on;
subplot(2,1,1)
plot(Estimated.DisE,Estimated.DisN)
grid on;
hold on;
plot(External.DisE,External.DisN,'r');
subplot(2,1,2);
plot(Estimated.DisE,Estimated.DisN);
grid on;
hold on;
plot(External.DisE,External.DisN,'r');

figure(2)
subplot(2,1,1)
plot(Rtk.timestamp(:)-Rtk.timestamp(1),Rtk.type);
subplot(2,1,2);
plot(NavSources.timestamp(:)-NavSources.timestamp(1),m_NavSources.maskValue);
figure(3)
% plot(EstimatedState.height-EstimatedState.z);
plot(Rtk.timestamp(1:end-1)-Rtk.timestamp(1),Rtk.timediff)

figure(3);
% subplot(2,1,1);
% plot3(Estimated.DisE,Estimated.DisN,Rtk.base_height(1)-Estimated.DisD,'b');
% grid on;
% hold on;
subplot(2,1,1)
plot(EstimatedState.timestamp(:)-EstimatedState.timestamp(1),EstimatedState.height-EstimatedState.z);
grid on;
hold on;
plot(DesiredZ.timestamp(:)-DesiredZ.timestamp(1),DesiredZ.value,'r');
plot(Rtk.timestamp(:)-Rtk.timestamp(1),Rtk.base_height(1)-Rtk.d,'-g');
plot( External.timestamp(:)-External.timestamp(1),External.base_height(1)-External.z,'c');
plot(Estimated.timestamp(:)-Estimated.timestamp(1),Rtk.base_height(1)-Estimated.DisD,'bl');
subplot(2,1,2);
plot(EstimatedState.timestamp(:)-EstimatedState.timestamp(1),EstimatedState.height-EstimatedState.z);
grid on;
hold on;
plot(DesiredZ.timestamp(:)-DesiredZ.timestamp(1),DesiredZ.value,'r');
plot(Rtk.timestamp(:)-Rtk.timestamp(1),Rtk.base_height(1)-Rtk.d,'-g');
plot( External.timestamp(:)-External.timestamp(1),External.base_height(1)-External.z,'c');
plot(Estimated.timestamp(:)-Estimated.timestamp(1),Rtk.base_height(1)-Estimated.DisD,'bl');

% plot(ExternalNed.timestamp(:)-ExternalNed.timestamp(1),External.base_height-External.z,'g')
% figure(2);
% % plot(EstimatedState.y,EstimatedState.x);
% plot(ExternalNed.y-2,ExternalNed.x-2);
% hold on;
% plot(Rtk.e(1,:),Rtk.n(1,:),'r');
% % plot(External.y,External.x,'g');
% plot(ExternalNed.y-diff*mean(Error.y.Data(1,1,98:100)),ExternalNed.x-diff*mean(Error.x.Data(1,1,98:100)),'c');
% legend('External','Rtk','ExternalDiff');
% y = squeeze(ExternalNed.timeY.Data)-squeeze(Error.y.Data);
% x = squeeze(ExternalNed.timeX.Data)-squeeze(Error.x.Data);
% plot(y,x,'k');
% figure(3);
% subplot(2,1,1);
% plot(Error.x);
% subplot(2,1,2);
% plot(Error.y);
% figure(4)
% plot(ExternalNed.y-2,ExternalNed.x-2);
% hold on;
% plot(GpsNed.y,GpsNed.x,'r');
% figure(5)
% plot(Rtk.timestamp(:) - Rtk.timestamp(1), Rtk.type);
% % ExternalNavData.state{2,1}