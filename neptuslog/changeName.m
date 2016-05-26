load ag20160525logs_hexaRTK/2hw/ntnu-hexa-004/135600_formationPlan/mra/data;

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
RtkHexa4 = struct;
RtkHexa4.n = zeros(1,sizeOfRtk);
RtkHexa4.e = zeros(1,sizeOfRtk);
RtkHexa4.d = zeros(1,sizeOfRtk);
RtkHexa4.type = zeros(1,sizeOfRtk);
RtkHexa4.base_lat = zeros(1,sizeOfRtk);
RtkHexa4.base_lon = zeros(1,sizeOfRtk);
RtkHexa4.base_height = zeros(1,sizeOfRtk);
RtkHexa4.timestamp = zeros(1,sizeOfRtk);
RtkHexa4.timediff = zeros(1,sizeOfRtk-1);
j = 1;
for i=1:length(GpsFixRtk.n)
    if (GpsFixRtk.src_ent(i)==src_ent)
        RtkHexa4.n(1,j) = GpsFixRtk.n(i);
        RtkHexa4.e(1,j) = GpsFixRtk.e(i);
        RtkHexa4.d(1,j) = GpsFixRtk.d(i);
        RtkHexa4.timestamp(1,j) = GpsFixRtk.timestamp(i);
        RtkHexa4.base_lon(1,j) = GpsFixRtk.base_lon(i)*180/pi;
        RtkHexa4.base_lat(1,j) = GpsFixRtk.base_lat(i)*180/pi;
        RtkHexa4.base_height(1,j) = GpsFixRtk.base_height(i);
        if strcmp(GpsFixRtk.type(i,1:2),'FI')
            RtkHexa4.type(j) = 3;
        elseif strcmp(GpsFixRtk.type(i,1:2),'FL')
            RtkHexa4.type(j) = 2;
        else
            RtkHexa4.type(j) = 0;
        end
        j = j+1;
    end
end
for i=1:sizeOfRtk-1
    RtkHexa4.timediff(i) = RtkHexa4.timestamp(i+1)-RtkHexa4.timestamp(i);
end
RtkHexa4.timeN = timeseries(RtkHexa4.n,RtkHexa4.timestamp);
RtkHexa4.timeE = timeseries(RtkHexa4.e,RtkHexa4.timestamp);
RtkHexa4.timeD = timeseries(RtkHexa4.d,RtkHexa4.timestamp);

%% Extract External

ExternalHexa4 = struct;

ExternalHexa4.x = zeros(1,sizeOfExternal);
ExternalHexa4.y = zeros(1,sizeOfExternal);
ExternalHexa4.z = zeros(1,sizeOfExternal);
ExternalHexa4.base_lat = zeros(1,sizeOfExternal);
ExternalHexa4.base_lon = zeros(1,sizeOfExternal);
ExternalHexa4.base_height = zeros(1,sizeOfExternal);
ExternalHexa4.timestamp = ExternalNavData.timestamp;
ExternalHexa4.DisN = zeros(1,length(ExternalNavData.timestamp));
ExternalHexa4.DisE = zeros(1,length(ExternalNavData.timestamp));
ExternalHexa4.DisD = zeros(1,length(ExternalNavData.timestamp));
for i=1:sizeOfExternal
    ExternalHexa4.x(i) = ExternalNavData.state{i,1}.x;
    ExternalHexa4.y(i) = ExternalNavData.state{i,1}.y;
    ExternalHexa4.z(i) = ExternalNavData.state{i,1}.z;
    ExternalHexa4.base_lat(i) = ExternalNavData.state{i,1}.lat*180/pi;
    ExternalHexa4.base_lon(i) = ExternalNavData.state{i,1}.lon*180/pi;
    ExternalHexa4.base_height(i) = ExternalNavData.state{i,1}.height;
    [ExternalHexa4.DisN(i),ExternalHexa4.DisE(i),ExternalHexa4.DisD(i)] = displacement(ExternalHexa4.base_lat(i),ExternalHexa4.base_lon(i),ExternalHexa4.base_height(i),RtkHexa4.base_lat(1),RtkHexa4.base_lon(1),RtkHexa4.base_height(1),ExternalHexa4.x(i),ExternalHexa4.y(i),ExternalHexa4.z(i));
end
ExternalHexa4.timeX = timeseries(ExternalHexa4.x,ExternalHexa4.timestamp);
ExternalHexa4.timeY = timeseries(ExternalHexa4.y,ExternalHexa4.timestamp);
ExternalHexa4.timeZ = timeseries(ExternalHexa4.z,ExternalHexa4.timestamp);

%% Extract Navsource used in system
m_NavSourcesHexa4 = struct;
% m_NavSources.mask = zeros(length(NavSources.mask),1);
m_NavSourcesHexa4.maskValue = zeros(length(NavSources.mask),1);
for i=1:length(NavSources.mask)
%     [m_NavSources.mask(i,:),~] = strsplit(NavSources.mask(i,:),{'GNSS_RTK','|'},'CollapseDelimiters',false,'DelimiterType','RegularExpression');
      index = strfind(NavSources.mask(i,:),'GNSS_RTK');
      m_NavSourcesHexa4.mask= NavSources.mask(1,index:index+7);
    if (strcmp(m_NavSourcesHexa4.mask,'GNSS_RTK'))
        m_NavSourcesHexa4.maskValue(i,1) = 1;
    else
        m_NavSourcesHexa4.maskValue(i,1) = 0;
    end
end

%% Extract estimatedState
EstimatedHexa4 = struct;

EstimatedHexa4.x = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.y = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.z = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.base_lat = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.base_lon = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.base_height = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.timestamp = EstimatedState.timestamp;
EstimatedHexa4.DisN = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.DisE = zeros(1,length(EstimatedState.timestamp));
EstimatedHexa4.DisD = zeros(1,length(EstimatedState.timestamp));
for i=1:length(EstimatedState.timestamp)
    EstimatedHexa4.x(i) = EstimatedState.x(i);
    EstimatedHexa4.y(i) = EstimatedState.y(i);
    EstimatedHexa4.z(i) = EstimatedState.z(i);
    EstimatedHexa4.base_lat(i) = EstimatedState.lat(i)*180/pi;
    EstimatedHexa4.base_lon(i) = EstimatedState.lon(i)*180/pi;
    EstimatedHexa4.base_height(i) = EstimatedState.height(i);
    [EstimatedHexa4.DisN(i),EstimatedHexa4.DisE(i),EstimatedHexa4.DisD(i)] = displacement(EstimatedHexa4.base_lat(i),EstimatedHexa4.base_lon(i),EstimatedHexa4.base_height(i),RtkHexa4.base_lat(1),RtkHexa4.base_lon(1),RtkHexa4.base_height(1),EstimatedHexa4.x(i),EstimatedHexa4.y(i),EstimatedHexa4.z(i));
end


%% Sync timeseries
[ExternalHexa4.timeX,RtkHexa4.timeN] = synchronize(ExternalHexa4.timeX,RtkHexa4.timeN,'Union');
[ExternalHexa4.timeY,RtkHexa4.timeE] = synchronize(ExternalHexa4.timeY,RtkHexa4.timeE,'Union');
[ExternalHexa4.timeZ,RtkHexa4.timeD] = synchronize(ExternalHexa4.timeZ,RtkHexa4.timeD,'Union');