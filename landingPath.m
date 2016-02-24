close all;
clear;


%% Diverse konstanter
deg2rad = pi/180;
rad2deg = 180/pi;

%% Input parametere fra Neptus
nett = [0 0 3]';
alpha = 0;
descent = 3;
a1 = 10;
a2 = 300;
a3 = 100;
Rl = 40;
nettH = 90*deg2rad;

%% UAV init poses in NED frame
x0 = [100 200 30 0 0 0]';

%% Nett in net frame
w1 = nett;
w2 = nett + [a1 0 a1*tan(alpha*deg2rad)]';
w3 = w2 + [a2 0 a2*tan(descent*deg2rad)]';
w4 = w3 + [a3 0 0]';
% Rotation matrix from nett to NED
R = [cos(nettH) -sin(nettH) 0;sin(nettH) cos(nettH) 0;0 0 1];

loiter1 = w4 + [0 Rl 0]';
loiter2 = w4 + [0 -Rl 0]';
WP = [w1 w2 w3 w4];
WPNED = [R*w1 R*w2 R*w3 R*w4];
loiter1NED = R*loiter1;
loiter2NED = R*loiter2;
figure(1);
plot3(WP(2,:),WP(1,:),WP(3,:));
hold on;
plot3(loiter1(2,:),loiter1(1,:),loiter1(3,:),'x');
plot3(loiter2(2,:),loiter2(1,:),loiter2(3,:),'x');
figure(2);
plot(WPNED(2,:),WPNED(1,:));
figure(3)
plot3(WPNED(2,:),WPNED(1,:),WPNED(3,:));
hold on;
plot3(loiter1NED(2,:),loiter1NED(1,:),loiter1NED(3,:),'x');
plot3(loiter2NED(2,:),loiter2NED(1,:),loiter2NED(3,:),'x');
plot3(x0(2),x0(1),x0(3),'x');