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
DescentBoxL = 100;

%% UAV init poses in NED frame
x0 = [500 200 0 0 0 0]';
p0 = x0(1:2);
p01 = p0 +[2*cos(x0(6)*deg2rad);2*sin(x0(6)*deg2rad)];
p01 = 10*(p01/norm(p01));

%% UAV spesific constants
R_min = 20;
K_max = 1/R_min;

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
gamma = [gammaT1 gammaT2 gammaT3 gammaT4];
[LT,IT] = min(T);
if mod(IT,2)==0
    [Lnett,Inett] = max([-1 Tnett(2) -1 Tnett(4)]);
else
    [Lnett,Inett] = max([Tnett(1) -1 Tnett(3) -1]);
end
Path = x0(1:3);
if abs(gamma(Inett)*rad2deg)>3
    if gamma(Inett)<0
        WPGlide = -descent;
    else
        WPGlide = descent;
    end
    gotoT = false;
    TPB1bx = Tmerk(1,Inett);
    TPB1ax = x0(1)-TPB1bx;
    TPB1by = Tmerk(2,Inett);
    TPB1ay = x0(2)-TPB1by;
    Tb = sqrt(DescentBoxL^2/(TPB1ax^2+TPB1ay^2));
    TPB1merk = [TPB1ay*Tb+TPB1bx;-TPB1ax*Tb+TPB1by;Tmerk(3,Inett)];
    Tb = -sqrt(DescentBoxL^2/(TPB1ax^2+TPB1ay^2));
    TPB2merk = [TPB1ay*Tb+TPB1bx;-TPB1ax*Tb+TPB1by;Tmerk(3,Inett)];
    Tb = sqrt(DescentBoxW^2/(TPB1ax^2+TPB1ay^2));
%     if abs(x0(3)-100*tan(descent*deg2rad))<Tmerk(3,Inett)
%         % Move towards T
%         WPB4 = [TPB1ax*Tb+TPB2merk(1);TPB1ay*Tb+TPB2merk(2);x0(3)];
%         Tb = sqrt((Tmerk(3,Inett)/tan(descent*deg2rad))^2/(TPB1ax^2+TPB1ay^2));
%         WPB1 = [TPB1ax*Tb+TPB1merk(1);TPB1ay*Tb+TPB1merk(2);Tmerk(3,Inett)];
%         Path = [x0(1:3) WPB4 WBB1 Tmerk(:,Inett)];
%         gotoT = true;
%     else
    WPB1 = [TPB1ax*Tb+TPB1merk(1);TPB1ay*Tb+TPB1merk(2);x0(3)+100*tan(WPGlide*deg2rad)];
    WPB2 = [TPB1ax*Tb+TPB2merk(1);TPB1ay*Tb+TPB2merk(2);x0(3)];
    Tb = sqrt(4*DescentBoxW^2/(TPB1ax^2+TPB1ay^2));
    WPB3 = [TPB1ax*Tb+TPB1merk(1);TPB1ay*Tb+TPB1merk(2);WPB1(3)];
    WPB4 = [TPB1ax*Tb+TPB2merk(1);TPB1ay*Tb+TPB2merk(2);x0(3)];
    dWPB3 = sqrt((Tmerk(1,Inett)-WPB3(1))^2+(Tmerk(2,Inett)-WPB3(2))^2);
    dWPB2 = sqrt((Tmerk(1,Inett)-WPB2(1))^2+(Tmerk(2,Inett)-WPB2(2))^2);
    tt = 2;
    Path(:,tt) = WPB4;
    tt = 3;
    Path(:,tt) = WPB2;
    tt = 4;
    Path(:,tt) = WPB1;
    tt = 5;
    Path(:,tt) = WPB3;
    odd = true;
    while(~gotoT)
       if (mod(odd,2)==0 && abs(atan2(Tmerk(3,Inett)-WPB2(3),dWPB2))*rad2deg<3)
           gotoT = true;
       elseif (mod(odd,2)==1 && abs(atan2(Tmerk(3,Inett)-WPB3(3),dWPB3))*rad2deg<3)
           gotoT = true;
       else
           if odd
               tt = tt+1;
               WPB1 = [WPB1(1);WPB1(2);WPB2(3)+100*tan(WPGlide*deg2rad)];
               WPB3 = [WPB3(1);WPB3(2);WPB1(3)];
               Path(:,tt) = WPB1;
               tt = tt+1;
               Path(:,tt) = WPB3;
               odd = false;
           else
               tt = tt+1;
               WPB4 = [WPB4(1);WPB4(2);WPB3(3)+100*tan(WPGlide*deg2rad)];
               WPB2 = [WPB2(1);WPB2(2);WPB4(3)];
               Path(:,tt) = WPB4;
               tt = tt+1;
               Path(:,tt) = WPB2;
               odd = true;
           end
       end
       
    end
%     if abs(h-100*tan(descent*deg2rad))<Tmerk(3,Inett)
%     end
%     end
end
tt = tt+1;
Path(:,tt) = Tmerk(:,Inett);
% %% Initi Dubins
% 
% xs = x0(1);
% ys = x0(2);
% K_s = K_max;
% psi_s = x0(6)*deg2rad;
% pf = R*w4;
% xf = pf(1);
% yf = pf(2);
% K_f = K_max;
% psi_f = pipi(nettH + pi);
% start_node = [x0(1) x0(2) x0(3) x0(6)*deg2rad 0 0];
% end_node = [pf(1) pf(2) pf(3) psi_f 0 0];
% % dubin = dubinsParameters(start_node,end_node,R_min);
% % psi = 0:0.01:2*pi;
% % cs = [dubin.cs(1) + R_min*cos(psi);dubin.cs(1) + R_min*sin(psi)];
% % cf = [dubin.ce(1) + R_min*cos(psi);dubin.ce(2) + R_min*sin(psi)];
% %% Algorithm for Dubins path
% psi = 0:0.01:2*pi;
% 
% xcs = xs - 1/K_s*cos(psi_s+pi/2);
% ycs = ys - 1/K_s*sin(psi_s+pi/2);
% xcf = xf - 1/K_f*cos(psi_f+pi/2);
% ycf = yf - 1/K_f*sin(psi_f+pi/2);
% cs = [xcs + 1/K_s*cos(psi);ycs + 1/K_s*sin(psi)];
% cf = [xcf + 1/K_f*cos(psi);ycf + 1/K_f*sin(psi)];
% 
% csec = [xcf + (1/K_f+1/K_s)*cos(psi);ycf + (1/K_f+1/K_s)*sin(psi)];
% bx = xcs;
% ax = xcf-bx;
% by = ycs;
% ay = ycf-by;
% t = 0:0.01:1;
% cx = ax*t+bx;
% cy = ay*t+by;
% tpN = sqrt((1/K_f)^2/(ax^2+ay^2)); % TODO: There are two way to turn
% tTmerk = sqrt((1/K_f+1/K_s)^2/(ax^2+ay^2));
% PN = [ay*tpN+xcf;-(ax*tpN)+ycf];
% Tmerk = [ay*tTmerk+xcf;-(ax*tTmerk)+ycf];
% OSTbx = Tmerk(1);
% OSTby = Tmerk(2);
% OSTax = xcs - OSTbx;
% OSTay = ycs - OSTby;
% OST = [OSTax*t+OSTbx;OSTay*t+OSTby];
% turn = atan2(ycf-ycs,xcf-xcs) + pi/2;
% xchi = xcs + 1/K_s*cos(turn);
% ychi = ycs + 1/K_s*sin(turn);
% xn = xcf + 1/K_f*cos(turn);
% yn = ycf + 1/K_f*sin(turn);
%% Figures
figure(1);
plot3(WP(2,:),WP(1,:),WP(3,:));
hold on;
plot3(loiter1(2,:),loiter1(1,:),loiter1(3,:),'x');
plot3(loiter2(2,:),loiter2(1,:),loiter2(3,:),'x');
figure(2);
plot(WPNED(2,:),WPNED(1,:));
figure(3)
plot3(WPNED(2,:),WPNED(1,:),WPNED(3,:),'-x');
hold on;
plot3(loiter1NED(2,:),loiter1NED(1,:),loiter1NED(3,:),'x');
plot3(loiter2NED(2,:),loiter2NED(1,:),loiter2NED(3,:),'x');
plot3(x0(2),x0(1),x0(3),'o');
plot3(cl1NED(2,:),cl1NED(1,:),cl1NED(3,:));
plot3(cl2NED(2,:),cl2NED(1,:),cl2NED(3,:));
plot3(Tmerk1(2,:),Tmerk1(1,:),Tmerk1(3,:),'x');
plot3(Tmerk2(2,:),Tmerk2(1,:),Tmerk2(3,:),'x');
plot3(Tmerk3(2,:),Tmerk3(1,:),Tmerk3(3,:),'x');
plot3(Tmerk4(2,:),Tmerk4(1,:),Tmerk4(3,:),'x');
plot3(WPB1(2,:),WPB1(1,:),WPB1(3,:),'x');
plot3(WPB2(2,:),WPB2(1,:),WPB2(3,:),'x');
plot3(WPB3(2,:),WPB3(1,:),WPB3(3,:),'x');
plot3(WPB4(2,:),WPB4(1,:),WPB4(3,:),'x');
plot3(TPB1merk(2,:),TPB1merk(1,:),TPB1merk(3,:),'x');
plot3(TPB2merk(2,:),TPB2merk(1,:),TPB2merk(3,:),'x');
plot3(Path(2,:),Path(1,:),Path(3,:));
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
