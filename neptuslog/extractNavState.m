function [state] = extractNavState(filename,Hardware)
state = struct;
load(filename);
rad2deg = 180/pi;
deg2rad = pi/180;
if (Hardware)
    C = unique(GpsFixRtk.src_ent);

    for i=1:length(C)
        row = find(EntityInfo.id==C(i));
        if strcmp(EntityInfo.component(row(1,:),1:9),'Sensors.R')
            src_ent = C(i);
        end
    end

    sizeOfRtk = length(find(GpsFixRtk.src_ent==src_ent));

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
            elseif strcmp(GpsFixRtk.type(i,1:2),'FL')
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
    
    state.Rtk = Rtk;

    %% Extract Navsource used in system
    NavSources1 = struct;
    % m_NavSources.mask = zeros(length(NavSources.mask),1);
    NavSources1.maskValue = zeros(length(NavSources.mask),1);
    NavSources1.timestamp = NavSources.timestamp;
    for i=1:length(NavSources.mask)
    %     [m_NavSources.mask(i,:),~] = strsplit(NavSources.mask(i,:),{'GNSS_RTK','|'},'CollapseDelimiters',false,'DelimiterType','RegularExpression');
          index = strfind(NavSources.mask(i,:),'GNSS_RTK');
          NavSources1.mask= NavSources.mask(1,index:index+7);
        if (strcmp(NavSources1.mask,'GNSS_RTK'))
            NavSources1.maskValue(i,1) = 1;
        else
            NavSources1.maskValue(i,1) = 0;
        end
    end
end
state.Navsources = NavSources1;
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
Estimated.phi = EstimatedState.phi;
Estimated.theta = EstimatedState.theta;
Estimated.psi = EstimatedState.psi;
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
end
state.Estimated = Estimated;
%% Extract external nav data
External = struct;
sizeOfExternal = length(ExternalNavData.state);
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
    [External.DisN(i),External.DisE(i),External.DisD(i)] = displacement(External.base_lat(i),External.base_lon(i),External.base_height(i),...
                                                                        Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),...
                                                                        External.x(i),External.y(i),External.z(i));
end
state.External = External;
end