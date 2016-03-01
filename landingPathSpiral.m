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
nettH = 0*deg2rad;
DescentBoxW = 20;
DescentBoxS = 40;
DescentBoxL = 100;

%% UAV init poses in NED frame
x0 = [400 200 40 0 0 0]';
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
psi = 0:0.01:2*pi;
n = loiter1NED(1)+Rl*cos(psi);
e = loiter1NED(2)+Rl*sin(psi);
d = loiter1NED(3)+zeros(1,length(psi));
cl1NED = [n;e;d];
n = loiter2NED(1)+Rl*cos(psi);
e = loiter2NED(2)+Rl*sin(psi);
d = loiter2NED(3)+zeros(1,length(psi));
cl2NED = [n;e;d];

%% Find Tmerk
c1bx = loiter1NED(1);
c1ax = x0(1)-c1bx;
c1by = loiter1NED(2);
c1ay = x0(2)-c1by;
Tt = sqrt(Rl^2/(c1ax^2+c1ay^2));
Tmerk1 = [c1ay*Tt+c1bx;-c1ax*Tt+c1by;loiter1NED(3)];
cT1bx = Tmerk1(1);
cT1ax = x0(1)-cT1bx;
cT1by = Tmerk1(2);
cT1ay = x0(2)-cT1by;
cT1h = Tmerk1(3)-x0(3);
Tnett1bx = Tmerk1(1);
Tnett1ax = WPNED(1,1)-Tnett1bx;
Tnett1by = Tmerk1(2);
Tnett1ay = WPNED(2,1)-Tnett1by;
Tnett1 = sqrt(Tnett1ax^2+Tnett1ay^2);
T1 = sqrt(cT1ax^2+cT1ay^2);
gammaT1 = atan2(cT1h,sqrt(cT1ax^2+cT1ay^2));

c2bx = loiter2NED(1);
c2ax = x0(1)-c2bx;
c2by = loiter2NED(2);
c2ay = x0(2)-c2by;
Tt = sqrt(Rl^2/(c2ax^2+c2ay^2));
Tmerk2 = [c2ay*Tt+c2bx;-c2ax*Tt+c2by;loiter2NED(3)];
cT2bx = Tmerk2(1);
cT2ax = x0(1)-cT2bx;
cT2by = Tmerk2(2);
cT2ay = x0(2)-cT2by;
cT2h = Tmerk2(3)-x0(3);
Tnett2bx = Tmerk2(1);
Tnett2ax = WPNED(1,1)-Tnett2bx;
Tnett2by = Tmerk2(2);
Tnett2ay = WPNED(2,1)-Tnett2by;
Tnett2 = sqrt(Tnett2ax^2+Tnett2ay^2);
T2 = sqrt(cT2ax^2+cT2ay^2);
gammaT2 = atan2(cT2h,sqrt(cT2ax^2+cT2ay^2));

c3bx = loiter1NED(1);
c3ax = x0(1)-c3bx;
c3by = loiter1NED(2);
c3ay = x0(2)-c3by;
Tt = -sqrt(Rl^2/(c3ax^2+c3ay^2));
Tmerk3 = [c3ay*Tt+c3bx;-c3ax*Tt+c3by;loiter1NED(3)];
cT3bx = Tmerk3(1);
cT3ax = x0(1)-cT3bx;
cT3by = Tmerk3(2);
cT3ay = x0(2)-cT3by;
cT3h = Tmerk3(3)-x0(3);
Tnett3bx = Tmerk3(1);
Tnett3ax = WPNED(1,1)-Tnett3bx;
Tnett3by = Tmerk3(2);
Tnett3ay = WPNED(2,1)-Tnett3by;
Tnett3 = sqrt(Tnett3ax^2+Tnett3ay^2);
T3 = sqrt(cT3ax^2+cT3ay^2);
gammaT3 = atan2(cT3h,sqrt(cT3ax^2+cT3ay^2));

c4bx = loiter2NED(1);
c4ax = x0(1)-c4bx;
c4by = loiter2NED(2);
c4ay = x0(2)-c4by;
Tt = -sqrt(Rl^2/(c4ax^2+c4ay^2));
Tmerk4 = [c4ay*Tt+c4bx;-c4ax*Tt+c4by;loiter2NED(3)];
cT4bx = Tmerk4(1);
cT4ax = x0(1)-cT4bx;
cT4by = Tmerk4(2);
cT4ay = x0(2)-cT4by;
cT4h = Tmerk4(3)-x0(3);
Tnett4bx = Tmerk4(1);
Tnett4ax = WPNED(1,1)-Tnett4bx;
Tnett4by = Tmerk4(2);
Tnett4ay = WPNED(2,1)-Tnett4by;
Tnett4 = sqrt(Tnett4ax^2+Tnett4ay^2);
T4 = sqrt(cT4ax^2+cT4ay^2);
gammaT4 = atan2(cT4h,sqrt(cT4ax^2+cT4ay^2));

T = [T1 T2 T3 T4];
Tnett = [Tnett1 Tnett2 Tnett3 Tnett4];
Tmerk = [Tmerk1 Tmerk2 Tmerk3 Tmerk4];
c = [c1ax c2ax c3ax c4ax;...
    c1bx c2bx c3bx c4bx;...
    c1ay c2ay c3ay c4ay;...
    c1by c2by c3by c4by];
gamma = [gammaT1 gammaT2 gammaT3 gammaT4];
[LT,IT] = min(T);
if mod(IT,2)==0
    [Lnett,Inett] = max([-1 Tnett(2) -1 Tnett(4)]);
else
    [Lnett,Inett] = max([Tnett(1) -1 Tnett(3) -1]);
end
XWP4ax = x0(1)-WPNED(1,4);
XWP4ay = x0(2)-WPNED(2,4);
dXWP4 = sqrt(XWP4ax^2+XWP4ay^2);

Path = x0(1:3);
tt = 1;
if (abs(gamma(Inett)*rad2deg)>3)
    if gamma(Inett)<0
        WPGlide = -descent;
    else
        WPGlide = descent;
    end
    gotoT = false;
    if sqrt(c(1,Inett)^2+c(3,Inett)^2)<=DescentBoxS+R_min
        tw = sqrt((DescentBoxS+2*R_min)^2/(c(1,Inett)^2+c(3,Inett)^2));
    else
        tw = sqrt((DescentBoxS)^2/(c(1,Inett)^2+c(3,Inett)^2));
    end
    tc = sqrt((DescentBoxS+R_min)^2/(c(1,Inett)^2+c(3,Inett)^2));
    WPC = [c(1,Inett)*tc+c(2,Inett);c(3,Inett)*tc+c(4,Inett);x0(3)];
    WPS0 = [c(1,Inett)*tw+c(2,Inett);c(3,Inett)*tw+c(4,Inett);x0(3)];
    theta0 = atan2(WPS0(2)-WPC(2),WPS0(1)-WPC(1));
    thetat = 0:(2*pi)/(N-1):2*pi;
    xnn = WPC(1)+R_min*cos(theta0+thetat(2));
    ynn = WPC(2)+R_min*sin(theta0+thetat(2));
    D = sqrt((xnn-WPS0(1))^2+(ynn-WPS0(2))^2);
    znn = WPC(3)+D*tan(WPGlide*deg2rad);
    WPS1 = [xnn;ynn;znn];
    tt = tt +1;
    Path(:,tt) = WPS0;
    tt = tt + 1;
    Path(:,tt) = WPS1;
    n = 3;
    TmerkWPCbx = Tmerk(1,Inett);
    TmerkWPCax = WPC(1)-TmerkWPCbx;
    TmerkWPCby = Tmerk(2,Inett);
    TmerkWPCay = WPC(2)-TmerkWPCby;
    tExit = -sqrt(R_min^2/(TmerkWPCax^2+TmerkWPCay^2));
    cExit = [TmerkWPCay*tExit+WPC(1);-TmerkWPCax*tExit+WPC(2);WPS1(3)];
    DcExit = sqrt((Tmerk(1,Inett)-cExit(1))^2+(Tmerk(2,Inett)-cExit(2))^2);
    gammaExit = atan2(Tmerk(1,Inett)-cExit(3),DcExit);
    % Find exit WP. It's perpenticular to c
    while(~gotoT)
       if (abs(atan2(Tmerk(3,Inett)-cExit(3),DcExit))*rad2deg<3)
           % Create path to cExit at the same height
           theta0 = atan2(WPS1(2)-WPC(2),WPS1(1)-WPC(1))
           theta = atan2(cExit(2)-WPC(2),cExit(1)-WPC(1))
           thetat = 0:(pipi(theta-theta0))/(N-1):pipi(theta-theta0);
           turn = [WPC(1) + R_min*cos(theta0+thetat);WPC(2)+R_min*sin(theta0+thetat);ones(1,N)*cExit(3)];
           Path = [Path turn];
           tt = length(Path);
           gotoT = true;
       else
           WPS0 = WPS1;
           xnn = WPC(1)+R_min*cos(theta0+thetat(n));
           ynn = WPC(2)+R_min*sin(theta0+thetat(n));
           D = sqrt((xnn-WPS0(1))^2+(ynn-WPS0(2))^2);
           znn = WPS0(3)+D*tan(WPGlide*deg2rad);
           WPS1 = [xnn;ynn;znn];
           cExit(3) = WPS1(3);
           tt = tt+1;
           Path(:,tt) = WPS1;
           n = n+1;
           % n is periodic
           if n>N
               n = 1;
           end
       end
    end

% elseif dXWP4<2*R_min
%     
%     TPB1bx = Tmerk(1,Inett);
%     TPB1ax = x0(1)-TPB1bx;
%     TPB1by = Tmerk(2,Inett);
%     TPB1ay = x0(2)-TPB1by;
%     Tb = sqrt(DescentBoxL^2/(TPB1ax^2+TPB1ay^2));
%     TPB1merk = [TPB1ay*Tb+TPB1bx;-TPB1ax*Tb+TPB1by;Tmerk(3,Inett)];
%     Tb = -sqrt(DescentBoxL^2/(TPB1ax^2+TPB1ay^2));
%     TPB2merk = [TPB1ay*Tb+TPB1bx;-TPB1ax*Tb+TPB1by;Tmerk(3,Inett)];
%     Tb = sqrt(DescentBoxS^2/(TPB1ax^2+TPB1ay^2));
%     WPB2 = [TPB1ax*Tb+TPB2merk(1);TPB1ay*Tb+TPB2merk(2);x0(3)];
%     tt = 2;
%     Path(:,tt) = WPB2;
end
tt = tt+1;
Path(:,tt) = Tmerk(:,Inett);
if mod(Inett,2)==1
    loiterNED = loiter1NED;
else
    loiterNED = loiter2NED;
end
theta0 = atan2(Tmerk(2,Inett)-loiterNED(2),Tmerk(1,Inett)-loiterNED(1));
theta = atan2(WPNED(2,4)-loiterNED(2),WPNED(1,4)-loiterNED(1));
thetat = 0:(pipi(theta-theta0))/(N-1):pipi(theta-theta0);
turn = [loiterNED(1) + Rl*cos(theta0+thetat);loiterNED(2)+Rl*sin(theta0+thetat);ones(1,N)*WPNED(3,4)];
Path = [Path turn];
tt = length(Path)+1;
Path(:,tt) = WPNED(:,3);
tt = tt+1;
Path(:,tt) = WPNED(:,2);
tt = tt+1;
Path(:,tt) = WPNED(:,1);

%% Figures
figure(1);
plot3(WP(2,:),WP(1,:),WP(3,:));
hold on;
plot3(loiter1(2,:),loiter1(1,:),loiter1(3,:),'x');
plot3(loiter2(2,:),loiter2(1,:),loiter2(3,:),'x');
figure(2);
plot(WPNED(2,:),WPNED(1,:));
figure(3)
% plot3(WPNED(2,:),WPNED(1,:),WPNED(3,:),'-x');
hold on;
% plot3(loiter1NED(2,:),loiter1NED(1,:),loiter1NED(3,:),'x');
% plot3(loiter2NED(2,:),loiter2NED(1,:),loiter2NED(3,:),'x');
plot3(x0(2),x0(1),x0(3),'o');
plot3(cl1NED(2,:),cl1NED(1,:),cl1NED(3,:));
plot3(cl2NED(2,:),cl2NED(1,:),cl2NED(3,:));
% plot3(Tmerk1(2,:),Tmerk1(1,:),Tmerk1(3,:),'x');
% plot3(Tmerk2(2,:),Tmerk2(1,:),Tmerk2(3,:),'x');
% plot3(Tmerk3(2,:),Tmerk3(1,:),Tmerk3(3,:),'x');
% plot3(Tmerk4(2,:),Tmerk4(1,:),Tmerk4(3,:),'x');
% plot3(WPB1(2,:),WPB1(1,:),WPB1(3,:),'x');
% plot3(WPB2(2,:),WPB2(1,:),WPB2(3,:),'x');
% plot3(WPB3(2,:),WPB3(1,:),WPB3(3,:),'x');
% plot3(WPB4(2,:),WPB4(1,:),WPB4(3,:),'x');
% plot3(TPB1merk(2,:),TPB1merk(1,:),TPB1merk(3,:),'x');
% plot3(TPB2merk(2,:),TPB2merk(1,:),TPB2merk(3,:),'x');
plot3(Path(2,:),Path(1,:),Path(3,:),'-x');
% figure(4)
% plot(ys,xs,'x');
% hold on;
% 
% quiver(p0(2),p0(1),p01(2),p01(1));
% hold on;
% plot(ycs,xcs,'x');
% plot(yf,xf,'x');
% plot(ycf,xcf,'x');
% plot(cs(2,:),cs(1,:));
% plot(cf(2,:),cf(1,:));
% plot(csec(2,:),csec(1,:));
% plot(cy,cx);
% % plot(ychi,xchi,'o');
% % plot(yn,xn,'o');
% plot(PN(2),PN(1),'o');
% plot(Tmerk(2),Tmerk(1),'o');
% plot(OST(2,:),OST(1,:));
% figure(5)
% plot(cs(2,:),cs(1,:));
% hold on;
% plot(cf(2,:),cf(1,:));
% plot(pf(2),pf(1),'x');
