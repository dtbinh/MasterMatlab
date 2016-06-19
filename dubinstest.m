close all;
clear;


%% Diverse konstanter
deg2rad = pi/180;
rad2deg = 180/pi;

%% Input parametere fra Neptus
nett = [0 0 -3]';
alpha = 3;
descent = 3;
a0 = 80;
a1 = 100;
a2 = 300;
a3 = 10;
Rl = 75;
nettH = 60*deg2rad;

%% UAV init poses in NED frame
x0 = [500 50 -50 0 0 0*deg2rad]';
p0 = x0(1:2);
p01 = p0 +[2*cos(x0(6)*deg2rad);2*sin(x0(6)*deg2rad)];
p01 = 10*(p01/norm(p01));

%% UAV spesific constants
R_min = 75;
K_max = 1/R_min;
N = 32;

%% Nett in net frame
w1 = nett + [-a0 0 a1*tan(alpha*deg2rad)]';
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
WPNED = [R*w1 R*w2 R*w3 R*w4];
wA = R*wA;
XS = [x0(1) x0(2) x0(3) x0(6)];
XF = [WPNED(1,4) WPNED(2,4) WPNED(3,4) (nettH-pi)];
[Path,OF,RightF,success,lengthPath,heading] = dubinsPath(XS,XF,R_min,Rl,N);

    
[Path,correctHeight] = glideslope(Path,x0(3),WPNED(3,4),descent*deg2rad);

[Path,lengthSpiral] = glideSpiral(Path,OF,Rl,RightF,WPNED(3,4),correctHeight,N,descent*deg2rad);

lengthPath = [lengthPath(1:end-1) lengthPath(end)+lengthSpiral];
% figure(2)
% plot(lengthPath,-Path(3,1:end-1));
% grid on;
% xlabel('Path length [m]');
% ylabel('Height [m]');
% legend('Path height profile');
figure(3)
plot3(Path(2,:),Path(1,:),-Path(3,:));
grid on;
hold on;
plot3(WPNED(2,:),WPNED(1,:),-WPNED(3,:),'-x');
legend('Approach path','Landing path');
xlabel('East [m]');
ylabel('North [m]');
zlabel('Height [m]');
figure(4)
plot(Path(2,:),Path(1,:));
grid on;
hold on;
plot(WPNED(2,:),WPNED(1,:),'-gx');
plot(0,0,'rx')
plot(Path(2,1),Path(1,1),'co')
legend('Approach path','Landing path','Net position','Start position');
xlabel('East [m]');
ylabel('North [m]');
axis('equal')
figure(5)
plot(lengthPath,heading*rad2deg);
grid on;
legend('Path tangential');
ylabel('[deg]');
xlabel('Path length [m]');