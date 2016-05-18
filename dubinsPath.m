function [Path,OF,RightF,success,lengthPath] = dubinsPath(Xs,Xf,Rs,Rf,N)
% Create dubins path between two positions.
% Start pose
% Xs(1) = north (m)
% Xs(2) = east (m)
% Xs(3) = down (m)
% Xs(4) = heading (m)
% Rs turning circle start
% End pose
% Xf(1) = north (m)
% Xf(2) = east (m)
% Xf(3) = down (m)
% Xf(4) = heading (m)
% Rf turning circle end
% N Number of turing points

% Define start turning circle
% if atan2(Xs(2)-Xf(2),Xs(1)-Xf(2))<0
%     RightS = false;
%     Xsc = Xs(1)-Rs*cos(Xs(4)-pi/2);
%     Ysc = Xs(2)-Rs*sin(Xs(4)-pi/2);
% else
%     RightS = true;
%     Xsc = Xs(1)-Rs*cos(Xs(4)+pi/2);
%     Ysc = Xs(2)-Rs*sin(Xs(4)+pi/2);
% end
% 
% %Define end turning circle. First find whitch side
% if atan2(Xs(2)-Xf(2),Xs(1)-Xf(2))<0
%     RightF = false;
%     Xfc = Xf(1)-Rf*cos(Xf(4)-pi/2);
%     Yfc = Xf(2)-Rf*sin(Xf(4)-pi/2);
%      
% else
%     RightF = true;
%     Xfc = Xf(1)-Rf*cos(Xf(4)+pi/2);
%     Yfc = Xf(2)-Rf*sin(Xf(4)+pi/2);
% end
% % Radius of secound end circle
% Rsec = abs(Rf-Rs);
% 
% % Define c
% cbx = Xsc;
% cax = Xfc-cbx;
% cby = Ysc;
% cay = Yfc-cby;
% 
% dc = sqrt(cax^2+cay^2);
% 
% if abs(Rf-Rs)>abs(dc)
%     disp('Advarsel: Umulig Dubin');
%     success = false;
%     Path =0;
%     OF =0;
%     RightF =0;
%     return;
% end
% % Ensure that Xs is far enough away from Xf
% dXsXf = sqrt((Xs(1)-Xf(1))^2+(Xs(2)-Xf(2))^2);
% if dXsXf<2*Rf
%     disp('Warning: To close to finish waypoint');
%     success = false;
%     Path =0;
%     OF =0;
%     RightF =0;
%     return;
% end

% Tmerk is perpendicular to c form Of, and connect in Csec;
% ttmerk = sqrt(Rsec^2/(cax^2+cay^2));
% Tmerk = [cay*ttmerk+Xfc;-cax*ttmerk+Yfc];
% ttPn = sqrt(Rf^2/(cax^2+cay^2));
% PN = [cay*ttPn+cbx;-cax*ttPn+cby];
% OSTmerkbx = Xsc;
% OSTmerkax = Tmerk(1)-OSTmerkbx;
% OSTmerkby = Ysc;
% OSTmerkay = Tmerk(2)-OSTmerkby;
% 
% % Find the exit tangent point
% ttPchi = sqrt(Rs^2/(cax^2+cay^2));
% Pchi = [cay*ttPchi+cbx;-cax*ttPchi+cby];

% alpha = asin((Rf-Rs)/abs(dc));
% 
% beta = atan2(Yfc-Ysc,Xfc-Xsc);
% 
% thetas = turn(RightS,alpha,beta);
% 
% thetaf = turn(RightF,alpha,beta);
% 
% Pchi = [Xsc+Rs*cos(thetas);Ysc + Rs*sin(thetas)];
% PN = [Xfc+Rf*cos(thetaf);Yfc+Rf*sin(thetaf)];

%% Find shortest Dubin

%LL
RightS1 = true;
Xsc1 = Xs(1)-Rs*cos(Xs(4)+pi/2);
Ysc1 = Xs(2)-Rs*sin(Xs(4)+pi/2);

RightF1 = true;
Xfc1 = Xf(1)-Rf*cos(Xf(4)+pi/2);
Yfc1 = Xf(2)-Rf*sin(Xf(4)+pi/2);

Ocs = [Xsc1;Ysc1];
Ocf = [Xfc1;Yfc1];
TurnS = RightS1;
TurnF = RightF1;

[Pchi1,PN1] = dubinsParameters(Ocs,Ocf,Rs,Rf,TurnS,TurnF);
theta0 = atan2(Xs(2)-Ysc1,Xs(1)-Xsc1);
theta1 = atan2(Pchi1(2)-Ysc1,Pchi1(1)-Xsc1);
if pipi(theta1-theta0)<=0
    sltheta1 = abs(theta1-theta0);
else
    sltheta1 = 2*pi-abs(theta1-theta0);
end
theta01 = atan2(PN1(2)-Yfc1,PN1(1)-Xfc1);
theta11 = atan2(Xf(2)-Yfc1,Xf(1)-Xfc1);
if pipi(theta11-theta01)<=0
    fltheta1 = abs(theta11-theta01);
else
    fltheta1 = 2*pi-abs(theta11-theta01);
end


sDubin1 = Rs*sltheta1 + sqrt((Pchi1(1)-PN1(1))^2+(Pchi1(2)-PN1(2))^2)+Rf*fltheta1;

%LR
RightS2 = true;
Xsc2 = Xs(1)-Rs*cos(Xs(4)+pi/2);
Ysc2 = Xs(2)-Rs*sin(Xs(4)+pi/2);

RightF2 = false;
Xfc2 = Xf(1)-Rf*cos(Xf(4)-pi/2);
Yfc2 = Xf(2)-Rf*sin(Xf(4)-pi/2);

Ocs = [Xsc2;Ysc2];
Ocf = [Xfc2;Yfc2];
TurnS = RightS2;
TurnF = RightF2;

[Pchi2,PN2] = dubinsParameters(Ocs,Ocf,Rs,Rf,TurnS,TurnF);

theta0 = atan2(Xs(2)-Ysc2,Xs(1)-Xsc2);
theta1 = atan2(Pchi2(2)-Ysc2,Pchi2(1)-Xsc2);
if pipi(theta1-theta0)<=0
    sltheta2 = abs(theta1-theta0);
else
    sltheta2 = 2*pi-abs(theta1-theta0);
end
theta01 = atan2(PN2(2)-Yfc2,PN2(1)-Xfc2);
theta11 = atan2(Xf(2)-Yfc2,Xf(1)-Xfc2);
if pipi(theta11-theta01)>=0
    fltheta2 = abs(theta11-theta01);
else
    fltheta2 = 2*pi-abs(theta11-theta01);
end

sDubin2 = Rs*sltheta2 + sqrt((Pchi2(1)-PN2(1))^2+(Pchi2(2)-PN2(2))^2)+Rf*fltheta2;

% RL
RightS3 = false;
Xsc3 = Xs(1)-Rs*cos(Xs(4)-pi/2);
Ysc3 = Xs(2)-Rs*sin(Xs(4)-pi/2);

RightF3 = true;
Xfc3 = Xf(1)-Rf*cos(Xf(4)+pi/2);
Yfc3 = Xf(2)-Rf*sin(Xf(4)+pi/2);

Ocs = [Xsc3;Ysc3];
Ocf = [Xfc3;Yfc3];
TurnS = RightS3;
TurnF = RightF3;

[Pchi3,PN3] = dubinsParameters(Ocs,Ocf,Rs,Rf,TurnS,TurnF);

theta0 = atan2(Xs(2)-Ysc3,Xs(1)-Xsc3);
theta1 = atan2(Pchi3(2)-Ysc3,Pchi3(1)-Xsc3);
if pipi(theta1-theta0)>=0
    sltheta3 = abs(theta1-theta0);
else
    sltheta3 = 2*pi-abs(theta1-theta0);
end
theta01 = atan2(PN3(2)-Yfc3,PN3(1)-Xfc3);
theta11 = atan2(Xf(2)-Yfc3,Xf(1)-Xfc3);
if pipi(theta11-theta01)<=0
    fltheta3 = abs(theta11-theta01);
else
    fltheta3 = 2*pi-abs(theta11-theta01);
end

sDubin3 = Rs*sltheta3 + sqrt((Pchi3(1)-PN3(1))^2+(Pchi3(2)-PN3(2))^2)+Rf*fltheta3;

% RR
RightS4 = false;
Xsc4 = Xs(1)-Rs*cos(Xs(4)-pi/2);
Ysc4 = Xs(2)-Rs*sin(Xs(4)-pi/2);

RightF4 = false;
Xfc4 = Xf(1)-Rf*cos(Xf(4)-pi/2);
Yfc4 = Xf(2)-Rf*sin(Xf(4)-pi/2);

Ocs = [Xsc4;Ysc4];
Ocf = [Xfc4;Yfc4];
TurnS = RightS4;
TurnF = RightF4;

[Pchi4,PN4] = dubinsParameters(Ocs,Ocf,Rs,Rf,TurnS,TurnF);

theta0 = atan2(Xs(2)-Ysc4,Xs(1)-Xsc4);
theta1 = atan2(Pchi4(2)-Ysc4,Pchi4(1)-Xsc4);
if pipi(theta1-theta0)>=0
    sltheta4 = abs(theta1-theta0);
else
    sltheta4 = 2*pi-abs(theta1-theta0);
end
theta01 = atan2(PN4(2)-Yfc4,PN4(1)-Xfc4);
theta11 = atan2(Xf(2)-Yfc4,Xf(1)-Xfc4);
if pipi(theta11-theta01)>=0
    fltheta4 = abs(theta11-theta01);
else
    fltheta4 = 2*pi-abs(theta11-theta01);
end

sDubin4 = Rs*sltheta4 + sqrt((Pchi4(1)-PN4(1))^2+(Pchi4(2)-PN4(2))^2)+Rf*fltheta4;

XscV = [Xsc1 Xsc2 Xsc3 Xsc4];
YscV = [Ysc1 Ysc2 Ysc3 Ysc4];
XfcV = [Xfc1 Xfc2 Xfc3 Xfc4];
YfcV = [Yfc1 Yfc2 Yfc3 Yfc4];
PchiV = [Pchi1 Pchi2 Pchi3 Pchi4];
PNV = [PN1 PN2 PN3 PN4];

RightSV = [RightS1 RightS2 RightS3 RightS4];
RightFV = [RightF1 RightF2 RightF3 RightF4];

sDubinV = [sDubin1 sDubin2 sDubin3 sDubin4]
[Dub,Ind] = min(sDubinV)
% Ind = 4;

Xsc = XscV(Ind);
Ysc = YscV(Ind);
Xfc = XfcV(Ind);
Yfc = YfcV(Ind);
Pchi = PchiV(:,Ind);
PN = PNV(:,Ind);
RightS = RightSV(Ind);
RightF = RightFV(Ind);

%% Construct the path

theta0 = atan2(Xs(2)-Ysc,Xs(1)-Xsc);
theta1 = atan2(Pchi(2)-Ysc,Pchi(1)-Xsc);
if RightS
%     if ((pipi(theta1-pi)>=theta0 ) || (theta0>theta1 &&sign(theta1)==sign(theta0)))
    if pipi(theta1-theta0)<=0
        theta = 0:-(abs(pipi(theta1-theta0))/(N-1)):-abs(pipi(theta1-theta0));
    else
        theta = 0:-((2*pi-abs(pipi(theta1-theta0)))/(N-1)):-(2*pi-abs(pipi(theta1-theta0)));
    end
else
%     if ((pipi(theta1-pi)<=theta0 && sign(pipi(theta1-pi))==sign(theta0)) || (theta0<theta1 && (sign(theta1)==sign(theta0) || theta0==0)))
    if pipi(theta1-theta0)>=0
        theta = 0:abs((pipi(theta1-theta0))/(N-1)):abs(pipi(theta1-theta0));
    else
        theta = 0:(2*pi-abs((pipi(theta1-theta0))))/(N-1):2*pi-abs(pipi(theta1-theta0));
    end
    
end
% theta = 0:((theta1-theta0))/(N-1):theta1-theta0;
arc1 = [Xsc+Rs*cos(theta0+theta);Ysc+Rs*sin(theta0+theta)];
lengthPath = Rs*abs(theta);
Path = arc1;
theta01 = atan2(PN(2)-Yfc,PN(1)-Xfc);
theta11 = atan2(Xf(2)-Yfc,Xf(1)-Xfc);
% && sign(pipi(theta11-pi))==sign(theta01)

lengthPath = [lengthPath lengthPath(end)+sqrt((Pchi(1)-PN(1))^2+(Pchi(2)-PN(2))^2)];


if RightF
    %counter clock wise rotation
%     if ((pipi(theta11-pi)>=theta01 ) || (theta01>theta11 &&sign(theta11)==sign(theta01)))
    if pipi(theta11-theta01)<=0
        theta1 = 0:-(abs(pipi(theta11-theta01))/(N-1)):-abs(pipi(theta11-theta01));
    else
        theta1 = 0:-((2*pi-abs(pipi(theta11-theta01)))/(N-1)):-(2*pi-abs(pipi(theta11-theta01)));
    end
else
%     if ((pipi(theta11-pi)<=theta01 && sign(pipi(theta11-pi))==sign(theta01)) || (theta01<theta11 && sign(theta11)==sign(theta01)))
    if pipi(theta11-theta01)>=0
        theta1 = 0:abs((pipi(theta11-theta01))/(N-1)):abs(pipi(theta11-theta01));
    else
        theta1 = 0:(2*pi-abs((pipi(theta11-theta01))))/(N-1):2*pi-abs(pipi(theta11-theta01));
    end
end
% theta1 = 0:(theta11-theta01)/(N-1):theta11-theta01;
arc2 = [Xfc+Rf*cos(theta01+theta1);Yfc+Rf*sin(theta01+theta1)];
Path = [Path arc2];
lengthPath = [lengthPath(1:end-1) lengthPath(end)+Rf*abs(theta1)];
OF = [Xfc;Yfc];
success = true;

%% Creat tangential path

% tarc1 = [-Rs*sin(theta0+theta);Rs*cos(theta0+theta)];
% tPath = tarc1;
% straightL = [PN(1)-Pchi(1);PN(2)-Pchi(2)]*ones(1,N);
% tPath = [tPath straightL];
% tarc2 = [-Rf*sin(theta01+theta1);Rf*cos(theta01+theta1)];
% tPath = [tPath tarc2];

%% Debuging 
s = 0:0.01:1;
% OST = [OSTmerkax*s+OSTmerkbx;OSTmerkay*s+OSTmerkby];
figure(1);
plot(Path(2,:),Path(1,:));
hold on;
plot(Xs(2),Xs(1),'co');
plot(Xf(2),Xf(1),'cx');
% plot(OST(2,:),OST(1,:),'b');
% plot(Tmerk(2),Tmerk(1),'bx');
% plot(Ysc,Xsc,'cx');
plot(Pchi(2),Pchi(1),'rx');
plot(PN(2),PN(1),'ro')
axis equal
title('Dubins path');
xlabel('East [m]');
ylabel('North [m]');
grid on;
legend('Path','Start pose','Finish pose','Exit tangent point on start circle','Entry tangent point on finish cricle');

end