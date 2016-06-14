function [state] = stateExtract(Hardware,Path,filename,coop)
state = struct;
%% Extract state information
load(filename)
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
    numberFix = 0;
    numberFloat = 0;
    numberNone = 0;
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
                numberFix = numberFix +1;
            elseif strcmp(GpsFixRtk.type(i,1:2),'FL')
                Rtk.type(j) = 2;
                numberFloat = numberFloat +1;
            else
                Rtk.type(j) = 0;
                numberNone = numberNone + 1;
            end
            j = j+1;
        end
    end
    if numberFix==0
        disp('No fix');
    else
        disp('Fix presentage');
        numberFix/sizeOfRtk*100
    end
    if numberFloat==0
        disp('No float');
    else
        disp('Float presentage');
        numberFloat/sizeOfRtk*100
    end
    if numberNone==0
        disp('No none');
    else
        disp('None presentage');
        numberNone/sizeOfRtk*100
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
    state.Navsources = NavSources1;
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
    [Estimated.PathN(i),Estimated.PathE(i),Estimated.PathD(i)] = displacement(Estimated.base_lat(i),Estimated.base_lon(i),Estimated.base_height(i),...
                                                                                Path.NetPos.lat,Path.NetPos.lon,Path.NetPos.height-Path.NetPos.z,...
                                                                                Estimated.x(i),Estimated.y(i),Estimated.z(i));
end
state.Estimated = Estimated;

%% Extract exernal
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
    [External.DisN(i),External.DisE(i),External.DisD(i)] = displacement(External.base_lat(i),External.base_lon(i),External.base_height(i),Rtk.base_lat(1),Rtk.base_lon(1),Rtk.base_height(1),External.x(i),External.y(i),External.z(i));
end
External.timeX = timeseries(External.x,External.timestamp);
External.timeY = timeseries(External.y,External.timestamp);
External.timeZ = timeseries(External.z,External.timestamp);
state.External = External;

%% Calculate compensator
comp = struct;
[tempRTKN, tempEXTN] = synchronize(External.timeX,Rtk.timeN,'Union');
[tempRTKE, tempEXTE] = synchronize(External.timeY,Rtk.timeE,'Union');
[tempRTKD, tempEXTD] = synchronize(External.timeZ,Rtk.timeD,'Union');
comp.x = tempRTKN-tempEXTN;
comp.y = tempRTKE-tempEXTE;
comp.z = tempRTKD-tempEXTD;
state.comp = comp;
%% Extract path state

PathState = struct;

C = unique(PathControlState.src_ent);

for i=1:length(C)
row = find(EntityInfo.id==C(i));
    if strcmp(EntityInfo.component(row(1,:),1:21),'Control.Path.LOSnSMCu')
        src_ent = C(i);
    end
end

sizeOfPathState = length(find(PathControlState.src_ent==src_ent));
sizeOfDesiredRoll = length(find(DesiredRoll.src_ent==src_ent));

PathState.crossTrack = zeros(1,sizeOfPathState);
PathState.alongtrack = zeros(1,sizeOfPathState);
PathState.timestamp = zeros(1,sizeOfPathState);

%% Extract desired roll
m_DesiredRoll = struct;
m_DesiredRoll.value = DesiredRoll.value;
m_DesiredRoll.timestamp = DesiredRoll.timestamp;
m_DesiredRoll.value = zeros(1,sizeOfDesiredRoll);
m_DesiredRoll.timestamp = zeros(1,sizeOfDesiredRoll);
j = 1;
for (i=1:length(PathControlState.timestamp))
    if (PathControlState.src_ent(i)==src_ent)
        PathState.crossTrack(j) = PathControlState.y(i);
        PathState.alongtrack(j) = PathControlState.x(i);
        PathState.timestamp(j) = PathControlState.timestamp(i);
        m_DesiredRoll.value(j) = DesiredRoll.value(i);
        m_DesiredRoll.timestamp(j) = DesiredRoll.timestamp(i);
        
        j = j+1;
    end
end
j = 1;
for i=1:length(DesiredRoll.timestamp)
    if (DesiredRoll.src_ent(i)==src_ent)
        m_DesiredRoll.value(j) = DesiredRoll.value(i);
        m_DesiredRoll.timestamp(j) = DesiredRoll.timestamp(i);
        j = j+1;
    end
end
state.DesiredRoll = m_DesiredRoll;
C = unique(DesiredZ.src_ent);

for i=1:length(C)
    row = find(EntityInfo.id==C(i));
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
state.DesiredHeight = DesiredHeight;

C = unique(DesiredPitch.src_ent);
disp('Find desired pitch')
for i=1:length(C)
row = find(EntityInfo.id==C(i));
    if strcmp(EntityInfo.component(row(1,:),1:21),'Control.Path.Longitud')
        src_ent = C(i);
    end
end
%% Extract desired pitch
sizeOfDesiredPitch = length(find(DesiredPitch.src_ent==src_ent));

m_DesiredPitch = struct;
m_DesiredPitch.timestamp = zeros(1,sizeOfDesiredPitch);
m_DesiredPitch.value = zeros(1,sizeOfDesiredPitch);
j = 1;
for i=1:length(DesiredPitch.timestamp)
    if (DesiredPitch.src_ent(i)==src_ent)
        m_DesiredPitch.value(j) = DesiredPitch.value(i);
        m_DesiredPitch.timestamp(j) = DesiredPitch.timestamp(i);
        j = j+1;
    end 
end
state.DesiredPitch = m_DesiredPitch;

%% Find height error

if (~coop)
height = timeseries(state.Estimated.base_height-state.Estimated.z,state.Estimated.timestamp);
desired = timeseries(state.DesiredHeight.value,state.DesiredHeight.timestamp);
[heightSync,desiredSync] = synchronize(height,desired,'Union');
heightError = heightSync-desiredSync;
state.heightError = heightError;

state.PathState = PathState;
disp('Mean cross track error');
mean(state.PathState.crossTrack)
disp('Var cross track error');
var(state.PathState.crossTrack)
disp('Mean height error');
mean(state.heightError)
end
% %% Find cross track error and along track distance in the lateral plane
% 
% alpha_k = atan2(Path.PathY(2)-Path.PathY(1),Path.PathX(2)-Path.PathX(1));
% i = 1;
% j = 1;
% lengthPath = length(Path.PathY);
% lengthState = length(EstimatedState.timestamp);
% alongTrack = zeros(1,length(EstimatedState.timestamp));
% crossTrack = zeros(1,length(EstimatedState.timestamp));
% first = true;
% 
% while (i<(lengthPath-1) && j<lengthState)
%     alongTrack(j) = (Estimated.PathN(j)-Path.PathX(i))*cos(alpha_k) + (Estimated.PathE(j)-Path.PathY(i))*sin(alpha_k);
%     crossTrack(j) = -(Estimated.PathN(j)-Path.PathX(i))*sin(alpha_k) + (Estimated.PathE(j)-Path.PathY(i))*cos(alpha_k);
%     if (alongTrack(j) >= sqrt((Path.PathX(i+1)-Path.PathX(i))^2+(Path.PathY(i+1)-Path.PathY(i))^2))
%         i = i+1;
%         alpha_k = atan2(Path.PathY(i+1)-Path.PathY(i),Path.PathX(i+1)-Path.PathX(i));
%     end
%     j = j+1;
% end
% state.crossTrack = crossTrack;
m_DesiredSpeed = struct;
m_DesiredSpeed.value = DesiredSpeed.value;
m_DesiredSpeed.timestamp = DesiredSpeed.timestamp;

state.DesiredSpeed = m_DesiredSpeed;

m_airspeed = struct;
m_airspeed.value = IndicatedSpeed.value;
m_airspeed.timestamp = IndicatedSpeed.timestamp;

state.IndicatedSpeed = m_airspeed;

m_trueSpeed = struct;
m_trueSpeed.value = TrueSpeed.value;
m_trueSpeed.timestamp = TrueSpeed.timestamp;
state.TrueSpeed = m_trueSpeed;
end