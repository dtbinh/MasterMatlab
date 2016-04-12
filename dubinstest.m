close all;
clear;


%% Diverse konstanter
deg2rad = pi/180;
rad2deg = 160/pi;

%% Input parametere fra Neptus
nett = [0 0 -3]';
alpha = 3;
descent = 3;
a1 = 10;
a2 = 300;
a3 = 100;
Rl = 20;
nettH = 0*deg2rad;
DescentBoxW = 20;
DescentBoxS = 40;
DescentBoxL = 100;

%% UAV init poses in NED frame
x0 = [500 100 -40 0 0 0*deg2rad]';
p0 = x0(1:2);
p01 = p0 +[2*cos(x0(6)*deg2rad);2*sin(x0(6)*deg2rad)];
p01 = 10*(p01/norm(p01));

%% UAV spesific constants
R_min = 20;
K_max = 1/R_min;
N = 16;

%% Nett in net frame
% TODO: Move the nett inbetween two waypoints.
w1 = nett;
w2 = nett + [a1 0 -a1*tan(alpha*deg2rad)]';
w3 = w2 + [a2 0 -a2*tan(descent*deg2rad)]';
w4 = w3 + [a3 0 0]';

% Define auxiliary waypoint
wA = w2 + [a2/2 Rl -a2*tan(descent*deg2rad)]';
% Rotation matrix from nett to NED
R = [cos(nettH) -sin(nettH) 0;sin(nettH) cos(nettH) 0;0 0 1];
loiter1 = w4 + [0 Rl 0]';
loiter2 = w4 + [0 -Rl 0]';
WP = [w1 w2 w3 w4];
WPNED = [R*w1 R*w2 R*w3 R*w4]
wA = R*wA;
XS = [x0(1) x0(2) x0(3) x0(6)];
XF = [WPNED(1,4) WPNED(2,4) WPNED(3,4) (nettH-pi)];
[Path,OF,RightF,success] = dubinsPath(XS,XF,R_min,Rl,N);
if ~success
    % Need an extra WP
    XF = [wA' nettH];
    [Path1,~,~,~] = dubinsPath(XS,XF,R_min,Rl,N);
    XF = [WPNED(1,4) WPNED(2,4) WPNED(3,4) (nettH-pi)];
    XS = [wA' nettH];
    [Path2,OF,RightF,success] = dubinsPath(XS,XF,R_min,Rl,N);
    Path = [Path1 Path2];
end  
if ~success
    disp('Abort landing');
    return
end
    
    [Path,correctHeight] = glideslope(Path,x0(3),WPNED(3,4),descent*deg2rad);
    Path = glideSpiral(Path,OF,Rl,RightF,WPNED(3,4),correctHeight,N,descent*deg2rad);
    


    % tt = length(Path)+1;
    % Path(:,tt) = WPNED(:,3);
    % tt = tt+1;
    % Path(:,tt) = WPNED(:,2);
    % tt = tt+1;
    % Path(:,tt) = WPNED(:,1);
    figure(2)
    plot(-Path(3,:));
    figure(3)
    plot3(Path(2,:),Path(1,:),-Path(3,:));
    hold on;
    plot3(WPNED(2,:),WPNED(1,:),-WPNED(3,:),'-x');
    % plot(Path(2,:),Path(1,:));
    % hold on;
    % plot(WPNED(2,4),WPNED(1,4),'x');
    % plot(x0(2),x0(1),'o');
    % axis equal
    % plot(WPNED(2,:),WPNED(1,:));
